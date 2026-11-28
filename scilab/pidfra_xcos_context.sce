// ============================================================================
// PIDFRA_XCOS_CONTEXT.SCE — Variables de contexto para Scilab Xcos (2024.0.0)
// ============================================================================
// Uso en Xcos:
//   Opcion A: En la consola de Scilab ejecutar:
//             exec('scilab/pidfra_xcos_context.sce', -1);
//   Opcion B: En la ventana de Xcos ir a:
//             Simulation -> Set Context
//             y pegar este bloque (o escribir: exec('ruta/a/pidfra_xcos_context.sce', -1);)
// ============================================================================

// --- 1. Muestreo, encoder y actuador (include/app_config.h) ---
Ts       = 0.01;          // Periodo de control [s] (100 Hz)
N_c      = 3600;          // 900 PPR * 4 (cuadratura x4) [cuentas/vuelta]
DEG2CNT  = N_c / 360.0;   // 10 cuentas/grado
CNT2DEG  = 360.0 / N_c;   // 0.1 grados/cuenta
PWM_MAX  = 255;           // Resolucion PWM LEDC 8 bits
MINMOVE  = 185;           // Precompensacion de zona muerta (poner 0 para ver stick-slip)
TOL_CNT  = 8;             // Banda de tolerancia en destino (+/- 8 cuentas = +/- 0.8 deg)

// --- 2. Planta electromecanica continua (Motor DC + L298N) ---
tau_m     = 0.090;        // Constante de tiempo mecanica [s] (90 ms)
u_static  = 178.0;        // Umbral de friccion estatica / zona muerta [duty]
Km_rpm    = 0.75;         // Ganancia estacionaria en velocidad [RPM / duty efectivo]
Km_counts = Km_rpm * (N_c / 60.0); // 45.0 (cuentas/s) / duty efectivo

// --- 3. PID Discreto de POSICION (Euler regresivo: C(z) = (q0*z^2 + q1*z + q2)/(z^2 - z)) ---
Kp_p = 0.4000;
Ki_p = 0.2000;
Kd_p = 0.0150;
q0_p = Kp_p + Ki_p * Ts + Kd_p / Ts;
q1_p = -(Kp_p + 2.0 * Kd_p / Ts);
q2_p = Kd_p / Ts;

// --- 4. PID Discreto de MODO CLOCK (segundero: 60 divisiones, 6 deg/s = 60 cnt/s) ---
Kp_c = 1.5000;
Ki_c = 1.0000;
Kd_c = 0.0150;
q0_c = Kp_c + Ki_c * Ts + Kd_c / Ts;
q1_c = -(Kp_c + 2.0 * Kd_c / Ts);
q2_c = Kd_c / Ts;
CLOCK_STEP_CNT = N_c / 60.0; // 60 cuentas por segundo

// --- 5. PID Discreto de VELOCIDAD (Metodo M: d_rpm = 60 / (N_c * Ts) = 1.6667 RPM/cnt) ---
Kp_v = 0.1200;            // Valores de firmware (para 35 RPM con FF probar Kp_v=2.0, Ki_v=12.0)
Ki_v = 0.1000;
Kd_v = 0.0020;
q0_v = Kp_v + Ki_v * Ts + Kd_v / Ts;
q1_v = -(Kp_v + 2.0 * Kd_v / Ts);
q2_v = Kd_v / Ts;
RPM_PER_CNT = 60.0 / (N_c * Ts); // 1.6667 RPM por cuenta en ventana Ts

// --- 6. Filtro IIR EMA de potenciometros ADC1 (lib/pot_adc) ---
Tf_pot    = 0.050;        // Periodo pot_task = 50 ms
alpha_ema = 0.25;         // H_ema(z) = (alpha_ema * z) / (z - (1 - alpha_ema))

// --- 7. Referencia de prueba por defecto ---
SP_DEG = 180.0;            // Consigna angular de prueba [grados]
SP_CNT = SP_DEG * DEG2CNT; // 900 cuentas

printf("Contexto PIDFRA Xcos cargado: Ts=%.3fs, N_c=%d, MINMOVE=%d, SP=%.1f deg (%.0f cnt)\n", ...
       Ts, N_c, MINMOVE, SP_DEG, SP_CNT);
