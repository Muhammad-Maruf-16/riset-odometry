#include <stdio.h>
#include <math.h>
#include <ctype.h>

#define DEG2RAD(x) ((x) * M_PI / 180.0)
#define RAD2DEG(x) ((x) * 180.0 / M_PI)
#define MIN_POS_RES 1.0
#define MIN_DEG_RES 2.0

// Dummy fuzzy (anggap output = 1)
double FuzzyLinear(double s) {
    return 1.0;  // Absolut = 1
}

double FuzzyOmega(double w) {
    return 1.0;  // Absolut = 1
}

int main() {

    while (1) {

        double Xr, Yr, Tr;
        double Xt, Yt, Tt;

        char cmd;

        printf("\nMasukkan currentPOS (X Y T[deg]) atau q untuk keluar: ");
        if (scanf(" %c", &cmd) != 1) return 0;

        if (cmd == 'q' || cmd == 'Q') {
            printf("Keluar program.\n");
            break;
        }

        ungetc(cmd, stdin);
        scanf("%lf %lf %lf", &Xr, &Yr, &Tr);

        printf("Masukkan target (x y t[deg])     : ");
        scanf("%lf %lf %lf", &Xt, &Yt, &Tt);

        // ================= VAR =================
        double dx, dy;
        double s, dw;
        double v = 0, w = 0;
        double Vx, Vy;
        double Vx_local, Vy_local;
        double w1, w2, w3;
        double w1_v2, w2_v2, w3_v2;
        double cos_t, sin_t;

        // ================= RUMUS =================

        // Delta Position
        dx = Xt - Xr;
        dy = Yt - Yr;

        s = sqrt(dx * dx + dy * dy);
        dw = Tt - Tr;

        // Velocity linear
        if (s > MIN_POS_RES) {
            v = FuzzyLinear(s);
        }

        // Velocity angular
        if (fabs(dw) > MIN_DEG_RES) {
            w = 1.5 * FuzzyOmega(fabs(dw));
            if (dw < 0) {
                w *= -1;
            }
        }

        // Hitung kecepatan arah sumbu X dan Y (Vektor GLOBAL)
        if (s > 0.0) {
            Vx = v * dx / s;
            Vy = v * dy / s;
        } else {
            Vx = 0;
            Vy = 0;
        }

        // Precompute cos dan sin untuk transformasi
        cos_t = cos(DEG2RAD(Tr));
        sin_t = sin(DEG2RAD(Tr));

        // /*TRANSFORMASI GLOBAL TO LOCAL v1*/
        // double Vx_local_v1 = (sin_t * Vx) + (cos_t * Vy);
        // double Vy_local_v1 = (cos_t * Vx) - (sin_t * Vy);

        /*TRANSFORMASI GLOBAL TO LOCAL v2*/
        Vx_local = (cos_t * Vx) + (sin_t * Vy);
        Vy_local = (-sin_t * Vx) + (cos_t * Vy);

        // /*angular Motor [rad/s] v1 GLOBAL VELOCITY*/
        // w1 = (0.67 * Vx) + (0 * Vy) + (0.33 * w);
        // w2 = (-0.33 * Vx) + (0.58 * Vy) + (0.33 * w);
        // w3 = (-0.33 * Vx) + (-0.58 * Vy) + (0.33 * w);

        /*angular Motor [rad/s] v2 LOCAL VELOCITY*/
        w1_v2 = (0.667 * Vx_local) + (0.333 * w);
        w2_v2 = (-0.333 * Vx_local) + (0.577 * Vy_local) + (0.333 * w);
        w3_v2 = (-0.333 * Vx_local) + (-0.577 * Vy_local) + (0.333 * w);

        // w1 = (0.667 * Vx_local);
        // w2 = (-0.333 * Vx_local) + (0.577 * Vy_local);
        // w3 = (-0.333 * Vx_local) + (-0.577 * Vy_local);

        // ================= KUADRAN =================
        double qx = Xt - Xr;
        double qy = Yt - Yr;
        const char *quadrant;

        if (qx > 0 && qy > 0) quadrant = "I";
        else if (qx < 0 && qy > 0) quadrant = "II";
        else if (qx < 0 && qy < 0) quadrant = "III";
        else if (qx > 0 && qy < 0) quadrant = "IV";
        else quadrant = "Axis";

        // ================= OUTPUT =================
        printf("\n=== HASIL PERHITUNGAN ===\n");
        printf("Kuadran Target      : %s\n", quadrant);
        printf("dx, dy              : %.4f , %.4f\n", dx, dy);
        printf("s (jarak)           : %.4f\n", s);
        printf("dw (delta heading)  : %.4f deg\n", dw);
        printf("\n--- GLOBAL VELOCITY ---\n");
        printf("v (linear)          : %.4f\n", v);
        printf("w (angular)         : %.4f\n", w);
        printf("Vx, Vy (global)     : %.4f , %.4f\n", Vx, Vy);
        printf("\n--- LOCAL VELOCITY ---\n");
        printf("Vx_local, Vy_local  : %.4f , %.4f\n", Vx_local, Vy_local);
        printf("\n--- MOTOR ANGULAR VELOCITY [rad/s] ---\n");
        // printf("w1 (v1 global)      : %.4f\n", w1);
        // printf("w2 (v1 global)      : %.4f\n", w2);
        // printf("w3 (v1 global)      : %.4f\n", w3);
        printf("w1 (v2 local)       : %.4f\n", w1_v2);
        printf("w2 (v2 local)       : %.4f\n", w2_v2);
        printf("w3 (v2 local)       : %.4f\n", w3_v2);
    }

    return 0;
}
