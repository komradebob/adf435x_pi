/*
 * set_adf4351_tinker_spi2.c
 *
 * Tinker Board + ADF4351 via SPI2 + sysfs GPIO.
 *
 * Wiring:
 *   ADF4351 DATA  -> MOSI.2 (SPI2 MOSI), header pin 19
 *   ADF4351 CLK   -> SCLK.2 (SPI2 SCLK), header pin 23
 *   ADF4351 LE    -> GPIO6A1, header pin 35, SoC gpio-185
 *   ADF4351 MUX   -> GPIO7B0, header pin 37, SoC gpio-224
 *   ADF4351 CE    -> MUST be tied high (3.3 V). If CE is low, PLL is off.
 *
 * Uses:
 *   - /dev/spidev2.0 for SPI2. Change SPI_DEV below to /dev/spidev2.1
 *     if that is the node actually mapped to pins 19/23 on your image.
 *   - sysfs GPIO for LE and MUXOUT (185, 224).
 *
 * CLI:
 *   sudo ./set_adf4351_tinker_spi2 [ -R10 ]
 *                                  [ -P-4 | -P-1 | -P2 | -P5 ]
 *                                  [ -M ]
 *                                  <freq_khz>
 *
 *   -R10  : 10 MHz reference (default is 25 MHz)
 *   -P-4  : RF power -4 dBm
 *   -P-1  : RF power -1 dBm
 *   -P2   : RF power +2 dBm
 *   -P5   : RF power +5 dBm (default)
 *   -M    : mute RF output
 *
 * Exit codes:
 *   0 : success, PLL locked
 *   1 : error
 *   2 : programmed but PLL did not report lock
 */

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

/* ---------- sysfs GPIO helpers ---------- */

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

/* ---------- SPI write helper ---------- */

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

/* ---------- N / FRAC / MOD calculation ---------- */

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

/* ---------- Register calculation ---------- */

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

/* ---------- SPI / GPIO init and lock wait ---------- */

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

/* ---------------- main ---------------- */

int main(int argc, char *argv[])
{
    int use_ref10 = 0;
    long freq_khz = -1;
    int mute = 0;
    int out_power_code = 3; // 0..3 => -4, -1, +2, +5 dBm

    if (argc < 2) {
        fprintf(stderr,
          "Usage: %s [ -R10 ] [ -P-4 | -P-1 | -P2 | -P5 ] [ -M ] <freq_khz>\n",
          argv[0]);
        return 1;
    }

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-R10")) {
            use_ref10 = 1;
        } else if (!strcmp(argv[i], "-P-4")) {
            out_power_code = 0;
        } else if (!strcmp(argv[i], "-P-1")) {
            out_power_code = 1;
        } else if (!strcmp(argv[i], "-P2")) {
            out_power_code = 2;
        } else if (!strcmp(argv[i], "-P5")) {
            out_power_code = 3;
        } else if (!strcmp(argv[i], "-M")) {
            mute = 1;
        } else {
            freq_khz = strtol(argv[i], NULL, 10);
        }
    }

    if (freq_khz <= 0) {
        fprintf(stderr, "Missing or invalid frequency\n");
        fprintf(stderr,
          "Usage: %s [ -R10 ] [ -P-4 | -P-1 | -P2 | -P5 ] [ -M ] <freq_khz>\n",
          argv[0]);
        return 1;
    }

    double fout_hz = (double)freq_khz * 1000.0;
    double fref_hz = use_ref10 ? REF_10MHZ : REF_25MHZ;
    double fmhz = fout_hz / 1e6;

    if (fmhz < F_MIN_MHZ || fmhz > F_MAX_MHZ) {
        fprintf(stderr,
          "Requested frequency %.3f MHz out of [%.1f, %.1f] MHz\n",
          fmhz, F_MIN_MHZ, F_MAX_MHZ);
        return 1;
    }

    if (gpio_init() < 0) {
        gpio_close_all();
        return 1;
    }

    if (spi_init() < 0) {
        gpio_close_all();
        return 1;
    }

    uint32_t regs[6];
    if (adf4351_calc_regs(fout_hz, fref_hz, out_power_code, mute, regs) < 0) {
        close(spi_fd);
        gpio_close_all();
        return 1;
    }

    if (adf4351_write_reg(regs[5]) < 0) goto fail;
    if (adf4351_write_reg(regs[4]) < 0) goto fail;
    if (adf4351_write_reg(regs[3]) < 0) goto fail;
    if (adf4351_write_reg(regs[2]) < 0) goto fail;
    if (adf4351_write_reg(regs[1]) < 0) goto fail;
    if (adf4351_write_reg(regs[0]) < 0) goto fail;

    if (wait_for_lock() == 0) {
        printf("ADF4351 locked at approximately %.6f MHz (ref=%.2f MHz)\n",
               fout_hz / 1e6, fref_hz / 1e6);
        if (mute)
            printf("Output is muted.\n");
        else
            printf("Output power setting: %s dBm\n",
                (out_power_code == 0) ? "-4" :
                (out_power_code == 1) ? "-1" :
                (out_power_code == 2) ? "+2" : "+5");
        close(spi_fd);
        gpio_close_all();
        return 0;
    } else {
        fprintf(stderr, "Warning: PLL did not lock within %d ms\n",
                LOCK_TIMEOUT_MS);
        close(spi_fd);
        gpio_close_all();
        return 2;
    }

fail:
    perror("ADF4351 write");
    close(spi_fd);
    gpio_close_all();
    return 1;
}
