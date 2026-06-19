// adf4351_gui.c
//
// GTK3 GUI wrapper around your set_adf4351_tinker_spi2.c logic.
//
// - Enter frequency (MHz)
// - Select power (-4, -1, +2, +5 dBm)
// - Big red MUTED button
// - Red/green lock indicator
// - Preferences for startup behavior, stored in ~/.adf4351_gui.conf
//
// Build:
//   gcc -o adf4351_gui adf4351_gui.c `pkg-config --cflags --libs gtk+-3.0` -lm
//
// Run (likely as root for SPI/GPIO):
//   sudo ./adf4351_gui

#include <gtk/gtk.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

/* ------------------------------------------------------------------------
 * Original ADF4351 / TinkerBoard code (mostly unchanged)
 * ------------------------------------------------------------------------ */

#define SPI_DEV           "/dev/spidev2.0"  // change to "/dev/spidev2.1" if needed
#define SPI_SPEED_HZ      1000000
#define SPI_BITS_PER_WORD 8
#define SPI_MODE          SPI_MODE_0

#define LE_GPIO           185   // GPIO6A1, pin 35
#define MUXOUT_GPIO       224   // GPIO7B0, pin 37

#define F_MIN_MHZ         35.0
#define F_MAX_MHZ         4400.0

#define REF_25MHZ         25.0e6
#define REF_10MHZ         10.0e6

#define PFD_MAX_HZ        25.0e6

#define LOCK_TIMEOUT_MS   500
#define LOCK_POLL_MS      5

static int spi_fd = -1;
static int le_fd  = -1;
static int mux_fd = -1;

// ---------- sysfs GPIO helpers ----------

static int gpio_export(int gpio)
{
    int fd = open("/sys/class/gpio/export", O_WRONLY);
    if (fd < 0) {
        if (errno == EBUSY) return 0;
        perror("open /sys/class/gpio/export");
        return -1;
    }
    char buf[16];
    int len = snprintf(buf, sizeof(buf), "%d", gpio);
    if (write(fd, buf, len) != len) {
        if (errno != EBUSY) {
            perror("write export");
            close(fd);
            return -1;
        }
    }
    close(fd);
    return 0;
}

static int gpio_unexport(int gpio)
{
    int fd = open("/sys/class/gpio/unexport", O_WRONLY);
    if (fd < 0)
        return 0;
    char buf[16];
    int len = snprintf(buf, sizeof(buf), "%d", gpio);
    write(fd, buf, len);
    close(fd);
    return 0;
}

static int gpio_set_direction(int gpio, const char *dir)
{
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", gpio);
    int fd = open(path, O_WRONLY);
    if (fd < 0) {
        perror("open direction");
        return -1;
    }
    if (write(fd, dir, strlen(dir)) != (ssize_t)strlen(dir)) {
        perror("write direction");
        close(fd);
        return -1;
    }
    close(fd);
    return 0;
}

static int gpio_open_value_fd(int gpio, int for_write)
{
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", gpio);
    int flags = for_write ? O_WRONLY : O_RDONLY;
    int fd = open(path, flags);
    if (fd < 0) {
        perror("open value");
    }
    return fd;
}

static int le_set(int value)
{
    if (le_fd < 0) return -1;
    const char c = value ? '1' : '0';
    if (lseek(le_fd, 0, SEEK_SET) < 0) return -1;
    if (write(le_fd, &c, 1) != 1) {
        perror("write LE");
        return -1;
    }
    return 0;
}

static int mux_get(void)
{
    if (mux_fd < 0) return -1;
    char c;
    if (lseek(mux_fd, 0, SEEK_SET) < 0) return -1;
    if (read(mux_fd, &c, 1) != 1) {
        perror("read MUXOUT");
        return -1;
    }
    return (c == '0') ? 0 : 1;
}

// ---------- SPI write helper ----------

static int adf4351_write_reg(uint32_t word)
{
    uint8_t tx[4];
    tx[0] = (word >> 24) & 0xFF;
    tx[1] = (word >> 16) & 0xFF;
    tx[2] = (word >>  8) & 0xFF;
    tx[3] = (word      ) & 0xFF;

    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = 0,
        .len    = 4,
        .delay_usecs = 0,
        .speed_hz    = SPI_SPEED_HZ,
        .bits_per_word = SPI_BITS_PER_WORD,
    };

    if (le_set(0) < 0) return -1;
    if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) < 1) {
        perror("SPI_IOC_MESSAGE");
        return -1;
    }
    if (le_set(1) < 0) return -1;

    struct timespec ts = {0, 1000}; // ~1 µs
    nanosleep(&ts, NULL);
    return 0;
}

// ---------- N / FRAC / MOD calculation ----------

static int adf4351_calc_N_FRAC_MOD(double fvco, double fref_hz,
                                   int *p_R, int *p_INT, int *p_FRAC, int *p_MOD,
                                   int *p_prescaler_8_9)
{
    int R = 1;
    double fpfd = fref_hz / (double)R;
    while (fpfd > PFD_MAX_HZ && R < 1023) {
        R++;
        fpfd = fref_hz / (double)R;
    }
    if (fpfd < 1.0e3) {
        fprintf(stderr, "PFD too low (%.0f Hz)\n", fpfd);
        return -1;
    }

    double N = fvco / fpfd;
    int INT = (int)floor(N + 1e-10);
    double frac = N - (double)INT;

    int prescaler_8_9 = (INT >= 75) ? 1 : 0;
    if (INT < 23 || INT > 65535) {
        fprintf(stderr, "INT out of range: %d (fvco=%f, fpfd=%f)\n",
                INT, fvco, fpfd);
        return -1;
    }

    if (fabs(frac) < 1e-9) {
        *p_R = R;
        *p_INT = INT;
        *p_FRAC = 0;
        *p_MOD = 1;
        *p_prescaler_8_9 = prescaler_8_9;
        return 0;
    }

    static const int mod_candidates[] = {
        2,3,4,5,6,7,8,9,
        10,12,15,16,18,20,21,24,25,
        27,28,30,32,36,40,45,48,49,
        50,60,64,72,75,80,81,90,96,98,
        100,120,125,128,135,144,150,160,162,
        180,192,200,216,225,240,243,250,256,
        270,288,300,320,324,360,384,400,
        432,450,480,486,500,512,
        540,576,600,625,640,648,720,768,
        800,810,864,900,960,972,1000,
        1024,1200,1250,1280,1440,1500,1600,1620,
        1800,1920,2000,2048,2160,2250,2400,
        2500,2560,2700,2880,3000,3200,3240,
        3600,3750,3840,4000,4095
    };
    const int N_MOD_CANDS =
        (int)(sizeof(mod_candidates)/sizeof(mod_candidates[0]));

    double best_err = 1e9;
    int best_MOD = 1;
    int best_FRAC = 0;

    for (int i = 0; i < N_MOD_CANDS; i++) {
        int MOD = mod_candidates[i];
        if (MOD < 2 || MOD > 4095) continue;

        double frac_scaled = frac * (double)MOD;
        int FRAC = (int)round(frac_scaled);
        if (FRAC < 0) FRAC = 0;
        if (FRAC > MOD) FRAC = MOD;

        int test_INT = INT;
        if (FRAC == MOD) {
            test_INT += 1;
            FRAC = 0;
        }
        if (test_INT < 23 || test_INT > 65535) continue;

        double actual_N = (double)test_INT + (double)FRAC / (double)MOD;
        double actual_fvco = actual_N * fpfd;
        double err_hz = fabs(actual_fvco - fvco);
        if (err_hz < best_err) {
            best_err = err_hz;
            best_MOD = MOD;
            best_FRAC = FRAC;
            INT = test_INT;
        }
        if (best_err < 1.0) break;
    }

    *p_R = R;
    *p_INT = INT;
    *p_FRAC = best_FRAC;
    *p_MOD = best_MOD;
    *p_prescaler_8_9 = prescaler_8_9;
    return 0;
}

// ---------- Register calculation ----------

static int adf4351_calc_regs(double fout_hz, double fref_hz,
                             int out_power_code, int mute,
                             uint32_t regs[6])
{
    if (fout_hz < F_MIN_MHZ*1e6 || fout_hz > F_MAX_MHZ*1e6) {
        fprintf(stderr, "Frequency out of range\n");
        return -1;
    }

    int div_sel = 0;
    int div_factor = 1;
    double fvco = fout_hz;
    while (fvco < 2200e6 && div_factor < 64) {
        div_factor <<= 1;
        div_sel++;
        fvco = fout_hz * div_factor;
    }
    if (fvco < 2200e6 || fvco > 4400e6) {
        fprintf(stderr, "Unable to place VCO in 2.2–4.4 GHz range\n");
        return -1;
    }

    int ref_doubler = 0;
    int ref_div2    = 0;

    int R, INT, FRAC, MOD, prescaler_8_9;
    if (adf4351_calc_N_FRAC_MOD(fvco, fref_hz,
                                &R, &INT, &FRAC, &MOD, &prescaler_8_9) < 0)
        return -1;

    uint32_t r0 = 0;
    r0 |= (INT  & 0xFFFF) << 15;
    r0 |= (FRAC & 0x0FFF) << 3;
    regs[0] = r0;

    uint32_t r1 = 0;
    int phase = 1;
    r1 |= (phase & 0xFFF) << 15;
    r1 |= (MOD   & 0x0FFF) << 3;
    if (prescaler_8_9) r1 |= (1u << 27);
    r1 |= 0x1;
    regs[1] = r1;

    uint32_t r2 = 0;
    r2 |= (R & 0x3FF) << 14;
    if (ref_doubler) r2 |= (1u << 25);
    if (ref_div2)    r2 |= (1u << 24);
    r2 |= (6u << 20);           // MUXOUT = digital LD
    int cp_current = 0x8;
    r2 |= (cp_current & 0xF) << 9;
    r2 |= (3u << 29);           // low spur mode
    r2 |= (1u << 13);           // double buffer
    r2 |= 0x2;
    regs[2] = r2;

    uint32_t r3 = 0;
    int clkdiv = 1;
    r3 |= (clkdiv & 0xFFF) << 3;
    r3 |= 0x3;
    regs[3] = r3;

    uint32_t r4 = 0;
    r4 |= ((uint32_t)div_sel & 0x7) << 20;
    int band_sel_clk = 0x28;
    r4 |= (band_sel_clk & 0xFF) << 12;
    int out_power_bits = out_power_code & 0x3;
    r4 |= (out_power_bits & 0x3) << 3;
    if (!mute) r4 |= (1u << 5);
    r4 |= 0x4;
    regs[4] = r4;

    regs[5] = 0x580005;
    return 0;
}

// ---------- SPI / GPIO init and lock wait ----------

static int spi_init(void)
{
    uint8_t mode = SPI_MODE;
    uint8_t bits = SPI_BITS_PER_WORD;
    uint32_t speed = SPI_SPEED_HZ;

    spi_fd = open(SPI_DEV, O_RDWR);
    if (spi_fd < 0) {
        perror("open SPI_DEV");
        return -1;
    }
    if (ioctl(spi_fd, SPI_IOC_WR_MODE, &mode) == -1) {
        perror("SPI_IOC_WR_MODE");
        return -1;
    }
    if (ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits) == -1) {
        perror("SPI_IOC_WR_BITS_PER_WORD");
        return -1;
    }
    if (ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) == -1) {
        perror("SPI_IOC_WR_MAX_SPEED_HZ");
        return -1;
    }
    return 0;
}

static int gpio_init(void)
{
    if (gpio_export(LE_GPIO) < 0) return -1;
    if (gpio_export(MUXOUT_GPIO) < 0) return -1;
    usleep(200000);

    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d", LE_GPIO);
    if (access(path, F_OK) != 0) {
        fprintf(stderr, "GPIO %d not created at %s\n", LE_GPIO, path);
        return -1;
    }
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d", MUXOUT_GPIO);
    if (access(path, F_OK) != 0) {
        fprintf(stderr, "GPIO %d not created at %s\n", MUXOUT_GPIO, path);
        return -1;
    }

    if (gpio_set_direction(LE_GPIO, "out") < 0) return -1;
    if (gpio_set_direction(MUXOUT_GPIO, "in") < 0) return -1;

    le_fd  = gpio_open_value_fd(LE_GPIO, 1);
    if (le_fd < 0) return -1;
    mux_fd = gpio_open_value_fd(MUXOUT_GPIO, 0);
    if (mux_fd < 0) return -1;

    if (le_set(1) < 0) return -1;
    return 0;
}

static void gpio_close_all(void)
{
    if (le_fd  >= 0) { close(le_fd);  le_fd  = -1; }
    if (mux_fd >= 0) { close(mux_fd); mux_fd = -1; }
    gpio_unexport(LE_GPIO);
    gpio_unexport(MUXOUT_GPIO);
}

static int wait_for_lock(void)
{
    int elapsed = 0;
    while (elapsed < LOCK_TIMEOUT_MS) {
        int v = mux_get();
        if (v < 0) return -1;
        if (v == 1) return 0;
        usleep(LOCK_POLL_MS * 1000);
        elapsed += LOCK_POLL_MS;
    }
    return -1;
}

/* ------------------------------------------------------------------------
 * Convenience wrapper for GUI:
 *   program PLL for given output frequency (Hz), power code, mute
 *   and return lock status (0/1).
 * ------------------------------------------------------------------------ */

static double g_reference_hz = REF_25MHZ; // can be extended later if needed

static int adf4351_program(double fout_hz, int out_power_code, int mute,
                           int *locked_out)
{
    if (fout_hz <= 0.0) return -1;

    double fmhz = fout_hz / 1e6;
    if (fmhz < F_MIN_MHZ || fmhz > F_MAX_MHZ) {
        fprintf(stderr,
                "Requested frequency %.3f MHz out of [%.1f, %.1f] MHz\n",
                fmhz, F_MIN_MHZ, F_MAX_MHZ);
        return -1;
    }

    uint32_t regs[6];
    if (adf4351_calc_regs(fout_hz, g_reference_hz,
                          out_power_code, mute, regs) < 0) {
        return -1;
    }

    if (adf4351_write_reg(regs[5]) < 0) goto fail;
    if (adf4351_write_reg(regs[4]) < 0) goto fail;
    if (adf4351_write_reg(regs[3]) < 0) goto fail;
    if (adf4351_write_reg(regs[2]) < 0) goto fail;
    if (adf4351_write_reg(regs[1]) < 0) goto fail;
    if (adf4351_write_reg(regs[0]) < 0) goto fail;

    int locked = (wait_for_lock() == 0) ? 1 : 0;
    if (locked_out) *locked_out = locked;
    return 0;

fail:
    perror("ADF4351 write");
    if (locked_out) *locked_out = 0;
    return -1;
}

/* ------------------------------------------------------------------------
 * GUI + Preferences
 * ------------------------------------------------------------------------ */

#define CONFIG_FILE ".adf4351_gui.conf"

typedef struct {
    double  startup_frequency_mhz;
    double  last_frequency_mhz;
    int     startup_mode;        // 0 = last freq, 1 = fixed freq
    int     default_power_index; // 0..3 => -4, -1, +2, +5 dBm
    int     start_muted;         // 0/1
} AppPrefs;

typedef struct {
    GtkWidget   *window;
    GtkWidget   *freq_entry;
    GtkWidget   *power_combo;
    GtkWidget   *mute_button;
    GtkWidget   *lock_indicator;
    GtkWidget   *lock_label;

    AppPrefs    prefs;
    int         muted;           // 0/1
    int         locked;          // 0/1
} AppData;

/* --- config file helpers --- */

static const char *get_config_path(void)
{
    static char path[512];
    const char *home = getenv("HOME");
    if (!home) home = ".";
    snprintf(path, sizeof(path), "%s/%s", home, CONFIG_FILE);
    return path;
}

static void load_prefs(AppPrefs *p)
{
    p->startup_frequency_mhz = 100.0;
    p->last_frequency_mhz    = 100.0;
    p->startup_mode          = 1;     // fixed
    p->default_power_index   = 3;     // +5 dBm default
    p->start_muted           = 1;

    FILE *f = fopen(get_config_path(), "r");
    if (!f) return;

    char key[128], value[128];
    while (fscanf(f, "%127[^=]=%127s\n", key, value) == 2) {
        if (strcmp(key, "startup_frequency_mhz") == 0)
            p->startup_frequency_mhz = atof(value);
        else if (strcmp(key, "last_frequency_mhz") == 0)
            p->last_frequency_mhz = atof(value);
        else if (strcmp(key, "startup_mode") == 0)
            p->startup_mode = atoi(value);
        else if (strcmp(key, "default_power_index") == 0)
            p->default_power_index = atoi(value);
        else if (strcmp(key, "start_muted") == 0)
            p->start_muted = atoi(value);
    }

    fclose(f);
}

static void save_prefs(const AppPrefs *p)
{
    FILE *f = fopen(get_config_path(), "w");
    if (!f) return;

    fprintf(f, "startup_frequency_mhz=%f\n", p->startup_frequency_mhz);
    fprintf(f, "last_frequency_mhz=%f\n",    p->last_frequency_mhz);
    fprintf(f, "startup_mode=%d\n",          p->startup_mode);
    fprintf(f, "default_power_index=%d\n",   p->default_power_index);
    fprintf(f, "start_muted=%d\n",           p->start_muted);

    fclose(f);
}

/* --- lock indicator --- */

static void update_lock_indicator(AppData *app, int locked)
{
    app->locked = locked;

    const gchar *color = locked ? "green" : "red";
    const gchar *text  = locked ? "LOCKED" : "UNLOCKED";

    GdkRGBA rgba;
    gdk_rgba_parse(&rgba, color);

    gtk_widget_override_background_color(app->lock_indicator,
                                         GTK_STATE_FLAG_NORMAL,
                                         &rgba);
    gtk_label_set_text(GTK_LABEL(app->lock_label), text);
}

/* --- mapping between combo index and power code --- */

static int power_index_to_code(int idx)
{
    // 0..3 => -4, -1, +2, +5 dBm
    if (idx < 0) idx = 0;
    if (idx > 3) idx = 3;
    return idx;
}

static const char *power_index_to_text(int idx)
{
    switch (idx) {
    case 0: return "-4 dBm";
    case 1: return "-1 dBm";
    case 2: return "+2 dBm";
    case 3: return "+5 dBm";
    default: return "?";
    }
}

/* --- apply GUI state to hardware --- */

static void apply_ui_to_hw(AppData *app)
{
    const char *freq_text = gtk_entry_get_text(GTK_ENTRY(app->freq_entry));
    double fmhz = atof(freq_text);
    if (fmhz <= 0.0) return;

    gint idx = gtk_combo_box_get_active(GTK_COMBO_BOX(app->power_combo));
    int out_power_code = power_index_to_code(idx);

    int locked = 0;
    double fout_hz = fmhz * 1e6;

    if (adf4351_program(fout_hz, out_power_code, app->muted, &locked) == 0) {
        update_lock_indicator(app, locked);
        app->prefs.last_frequency_mhz = fmhz;
        save_prefs(&app->prefs);
    } else {
        update_lock_indicator(app, 0);
    }
}

/* --- signal handlers --- */

static void on_frequency_changed(GtkEditable *editable, AppData *app)
{
    apply_ui_to_hw(app);
}

static void on_power_changed(GtkComboBox *combo, AppData *app)
{
    gint idx = gtk_combo_box_get_active(combo);
    app->prefs.default_power_index = idx;
    save_prefs(&app->prefs);
    apply_ui_to_hw(app);
}

static void on_mute_toggled(GtkToggleButton *button, AppData *app)
{
    app->muted = gtk_toggle_button_get_active(button);
    const char *label = app->muted ? "MUTED" : "UNMUTED";
    gtk_button_set_label(GTK_BUTTON(button), label);
    apply_ui_to_hw(app);
}

/* --- Preferences dialog --- */

typedef struct {
    GtkWidget *dialog;
    GtkWidget *start_muted_check;
    GtkWidget *startup_mode_last;
    GtkWidget *startup_mode_fixed;
    GtkWidget *fixed_freq_entry;
    GtkWidget *default_power_combo;
} PrefsWidgets;

static void on_prefs_response(GtkDialog *d, gint response_id, PrefsWidgets *pw)
{
    if (response_id == GTK_RESPONSE_OK) {
        AppData *app = g_object_get_data(G_OBJECT(d), "appdata");
        AppPrefs *p  = &app->prefs;

        p->start_muted = gtk_toggle_button_get_active(
            GTK_TOGGLE_BUTTON(pw->start_muted_check));

        if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(pw->startup_mode_last)))
            p->startup_mode = 0;
        else
            p->startup_mode = 1;

        const char *f = gtk_entry_get_text(GTK_ENTRY(pw->fixed_freq_entry));
        p->startup_frequency_mhz = atof(f);

        p->default_power_index = gtk_combo_box_get_active(
            GTK_COMBO_BOX(pw->default_power_combo));

        save_prefs(p);
    }

    gtk_widget_destroy(GTK_WIDGET(d));
    g_free(pw);
}

static void on_menu_preferences(GtkMenuItem *item, AppData *app)
{
    PrefsWidgets *pw = g_new0(PrefsWidgets, 1);

    pw->dialog = gtk_dialog_new_with_buttons(
        "Preferences",
        GTK_WINDOW(app->window),
        GTK_DIALOG_MODAL | GTK_DIALOG_DESTROY_WITH_PARENT,
        "_Cancel", GTK_RESPONSE_CANCEL,
        "_OK",     GTK_RESPONSE_OK,
        NULL);

    GtkWidget *content = gtk_dialog_get_content_area(GTK_DIALOG(pw->dialog));
    GtkWidget *grid = gtk_grid_new();
    gtk_grid_set_row_spacing(GTK_GRID(grid), 6);
    gtk_grid_set_column_spacing(GTK_GRID(grid), 6);
    gtk_container_set_border_width(GTK_CONTAINER(grid), 10);
    gtk_container_add(GTK_CONTAINER(content), grid);

    // Start muted
    pw->start_muted_check = gtk_check_button_new_with_label("Always start muted");
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(pw->start_muted_check),
                                 app->prefs.start_muted);
    gtk_grid_attach(GTK_GRID(grid), pw->start_muted_check, 0, 0, 2, 1);

    // Startup frequency mode
    GtkWidget *startup_label = gtk_label_new("Startup frequency:");
    gtk_grid_attach(GTK_GRID(grid), startup_label, 0, 1, 1, 1);

    pw->startup_mode_last  = gtk_radio_button_new_with_label(NULL, "Last used");
    GSList *group = gtk_radio_button_get_group(GTK_RADIO_BUTTON(pw->startup_mode_last));
    pw->startup_mode_fixed = gtk_radio_button_new_with_label(group, "Fixed (MHz):");
    gtk_grid_attach(GTK_GRID(grid), pw->startup_mode_last,  0, 2, 1, 1);
    gtk_grid_attach(GTK_GRID(grid), pw->startup_mode_fixed, 0, 3, 1, 1);

    pw->fixed_freq_entry = gtk_entry_new();
    char buf[64];
    snprintf(buf, sizeof(buf), "%.3f", app->prefs.startup_frequency_mhz);
    gtk_entry_set_text(GTK_ENTRY(pw->fixed_freq_entry), buf);
    gtk_grid_attach(GTK_GRID(grid), pw->fixed_freq_entry, 1, 3, 1, 1);

    if (app->prefs.startup_mode == 0)
        gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(pw->startup_mode_last), TRUE);
    else
        gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(pw->startup_mode_fixed), TRUE);

    // Default power level
    GtkWidget *power_label = gtk_label_new("Default power level:");
    gtk_grid_attach(GTK_GRID(grid), power_label, 0, 4, 1, 1);

    pw->default_power_combo = gtk_combo_box_text_new();
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(pw->default_power_combo), "-4 dBm");
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(pw->default_power_combo), "-1 dBm");
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(pw->default_power_combo), "+2 dBm");
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(pw->default_power_combo), "+5 dBm");
    gtk_combo_box_set_active(GTK_COMBO_BOX(pw->default_power_combo),
                             app->prefs.default_power_index);
    gtk_grid_attach(GTK_GRID(grid), pw->default_power_combo, 1, 4, 1, 1);

    g_object_set_data(G_OBJECT(pw->dialog), "appdata", app);

    g_signal_connect(pw->dialog, "response",
                     G_CALLBACK(on_prefs_response), pw);

    gtk_widget_show_all(pw->dialog);
}

/* --- menu bar --- */

static GtkWidget* create_menu_bar(AppData *app)
{
    GtkWidget *menubar = gtk_menu_bar_new();

    GtkWidget *menu = gtk_menu_new();
    GtkWidget *root = gtk_menu_item_new_with_label("Settings");
    gtk_menu_item_set_submenu(GTK_MENU_ITEM(root), menu);
    gtk_menu_shell_append(GTK_MENU_SHELL(menubar), root);

    GtkWidget *prefs_item = gtk_menu_item_new_with_label("Preferences...");
    g_signal_connect(prefs_item, "activate",
                     G_CALLBACK(on_menu_preferences), app);
    gtk_menu_shell_append(GTK_MENU_SHELL(menu), prefs_item);

    GtkWidget *quit_item = gtk_menu_item_new_with_label("Quit");
    g_signal_connect_swapped(quit_item, "activate",
                             G_CALLBACK(gtk_widget_destroy), app->window);
    gtk_menu_shell_append(GTK_MENU_SHELL(menu), quit_item);

    return menubar;
}

/* --- main UI creation --- */

static void activate(GtkApplication *gapp, gpointer user_data)
{
    AppData *app = (AppData *)user_data;

    app->window = gtk_application_window_new(gapp);
    gtk_window_set_title(GTK_WINDOW(app->window), "ADF4351 Control");
    gtk_window_set_default_size(GTK_WINDOW(app->window), 420, 220);

    GtkWidget *vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 0);
    gtk_container_add(GTK_CONTAINER(app->window), vbox);

    GtkWidget *menubar = create_menu_bar(app);
    gtk_box_pack_start(GTK_BOX(vbox), menubar, FALSE, FALSE, 0);

    GtkWidget *grid = gtk_grid_new();
    gtk_grid_set_row_spacing(GTK_GRID(grid), 8);
    gtk_grid_set_column_spacing(GTK_GRID(grid), 8);
    gtk_container_set_border_width(GTK_CONTAINER(grid), 10);
    gtk_box_pack_start(GTK_BOX(vbox), grid, TRUE, TRUE, 0);

    // Frequency
    GtkWidget *freq_label = gtk_label_new("Frequency (MHz):");
    gtk_grid_attach(GTK_GRID(grid), freq_label, 0, 0, 1, 1);

    app->freq_entry = gtk_entry_new();
    gtk_grid_attach(GTK_GRID(grid), app->freq_entry, 1, 0, 1, 1);

    // Power
    GtkWidget *power_label = gtk_label_new("RF Power:");
    gtk_grid_attach(GTK_GRID(grid), power_label, 0, 1, 1, 1);

    app->power_combo = gtk_combo_box_text_new();
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(app->power_combo), "-4 dBm");
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(app->power_combo), "-1 dBm");
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(app->power_combo), "+2 dBm");
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(app->power_combo), "+5 dBm");
    gtk_grid_attach(GTK_GRID(grid), app->power_combo, 1, 1, 1, 1);

    // Mute button
    app->mute_button = gtk_toggle_button_new_with_label("MUTED");
    GdkRGBA red;
    gdk_rgba_parse(&red, "red");
    gtk_widget_override_background_color(app->mute_button,
                                         GTK_STATE_FLAG_NORMAL, &red);
    gtk_grid_attach(GTK_GRID(grid), app->mute_button, 0, 2, 2, 1);

    // Lock indicator
    GtkWidget *lock_box = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 6);
    gtk_grid_attach(GTK_GRID(grid), lock_box, 0, 3, 2, 1);

    GtkWidget *lock_text_label = gtk_label_new("Lock status:");
    gtk_box_pack_start(GTK_BOX(lock_box), lock_text_label, FALSE, FALSE, 0);

    app->lock_indicator = gtk_event_box_new();
    gtk_widget_set_size_request(app->lock_indicator, 20, 20);
    gtk_box_pack_start(GTK_BOX(lock_box), app->lock_indicator, FALSE, FALSE, 0);

    app->lock_label = gtk_label_new("UNKNOWN");
    gtk_box_pack_start(GTK_BOX(lock_box), app->lock_label, FALSE, FALSE, 0);

    // Connect signals
    g_signal_connect(app->freq_entry, "changed",
                     G_CALLBACK(on_frequency_changed), app);
    g_signal_connect(app->power_combo, "changed",
                     G_CALLBACK(on_power_changed), app);
    g_signal_connect(app->mute_button, "toggled",
                     G_CALLBACK(on_mute_toggled), app);

    // Initialize from prefs
    double start_freq_mhz;
    if (app->prefs.startup_mode == 0)
        start_freq_mhz = app->prefs.last_frequency_mhz;
    else
        start_freq_mhz = app->prefs.startup_frequency_mhz;

    char buf[64];
    snprintf(buf, sizeof(buf), "%.3f", start_freq_mhz);
 
 
    gtk_entry_set_text(GTK_ENTRY(app->freq_entry), buf);

    gtk_combo_box_set_active(GTK_COMBO_BOX(app->power_combo),
                             app->prefs.default_power_index);

    app->muted = app->prefs.start_muted ? 1 : 0;
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(app->mute_button),
                                 app->muted);

    // First hardware programming
    apply_ui_to_hw(app);

    gtk_widget_show_all(app->window);
}
int main(int argc, char **argv)
{
    GtkApplication *gapp;
    int status;
    AppData app = {0};

    // Load prefs before GUI starts
    load_prefs(&app.prefs);

    // Init GPIO + SPI once
    if (gpio_init() < 0) {
        gpio_close_all();
        fprintf(stderr, "GPIO init failed\n");
        return 1;
    }
    if (spi_init() < 0) {
        gpio_close_all();
        fprintf(stderr, "SPI init failed\n");
        return 1;
    }

    gapp = gtk_application_new("com.example.adf4351", G_APPLICATION_FLAGS_NONE);
    g_signal_connect(gapp, "activate", G_CALLBACK(activate), &app);

    status = g_application_run(G_APPLICATION(gapp), argc, argv);

    g_object_unref(gapp);

    // Clean up hardware
    if (spi_fd >= 0) close(spi_fd);
    gpio_close_all();

    return status;
}
