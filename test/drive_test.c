// SPDX-License-Identifier: LGPL-3.0
#include <hddc2b/functions/drive.h>
#include "common.h"

#include <math.h>

#define NUM_DRV       4
#define NUM_DRV_COORD 2
#define NUM_GND_COORD 2

static double wheel_distance[NUM_DRV] = {
    0.0775, 0.0775, 0.0775, 0.0775  // fl, rl, rr, fr
};

static double castor_offset[NUM_DRV] = {
    0.01, 0.01, 0.01, 0.01          // fl, rl, rr, fr
};


START_TEST(test_hddc2b_drv_frc_gnd_to_pvt)
{
    double f_whl[NUM_DRV * NUM_GND_COORD] = {
        -0.125,    0.125,           // fl-r, fl-l
         0.125,   -0.125,           // rl-r, rl-l
         0.125,    0.125,           // rr-r, rr-l
        -0.242863, 0.433052         // fr-r, fr-l
    };
    double f_drv[NUM_DRV * NUM_DRV_COORD];
    double res[NUM_DRV * NUM_DRV_COORD] = {
         0.25,      0.0,            // fl-x, fl-y
        -0.25,      0.0,            // rl-x, rl-y
         0.0,      -0.968817,       // rr-x, rr-y
         0.675915, -0.736979        // fr-x, fr-y
    };

    hddc2b_drv_frc_gnd_to_pvt(
            NUM_DRV,
            wheel_distance,
            castor_offset,
            f_whl,
            f_drv);

    for (int i = 0; i < NUM_DRV * NUM_DRV_COORD; i++) {
        ck_assert_dbl_eq(f_drv[i], res[i]);
    }
}
END_TEST


START_TEST(test_hddc2b_drv_frc_pvt_to_gnd)
{
    double f_drv[NUM_DRV * NUM_DRV_COORD] = {
         0.25,      0.0,            // fl-x, fl-y
        -0.25,      0.0,            // rl-x, rl-y
         0.0,      -0.968817,       // rr-x, rr-y
         0.675915, -0.736979        // fr-x, fr-y
    };
    double f_whl[NUM_DRV * NUM_GND_COORD];
    double res[NUM_DRV * NUM_GND_COORD] = {
        -0.125,    0.125,           // fl-r, fl-l
         0.125,   -0.125,           // rl-r, rl-l
         0.125,    0.125,           // rr-r, rr-l
        -0.242863, 0.433052         // fr-r, fr-l
    };

    hddc2b_drv_frc_pvt_to_gnd(
            NUM_DRV,
            wheel_distance,
            castor_offset,
            f_drv,
            f_whl);

    for (int i = 0; i < NUM_DRV * NUM_GND_COORD; i++) {
        ck_assert_dbl_eq(f_whl[i], res[i]);
    }
}
END_TEST


START_TEST(test_hddc2b_drv_vel_gnd_to_pvt)
{
    double xd_whl[NUM_DRV * NUM_GND_COORD] = {
        -1.0,  1.0,                 // fl-r, fl-l
         1.0, -1.0,                 // rl-r, rl-l
         1.0,  1.0,                 // rr-r, rr-l
        -1.0, -1.0                  // fr-r, fr-l
    };
    double xd_drv[NUM_DRV * NUM_DRV_COORD];
    double res[NUM_DRV * NUM_DRV_COORD] = {
         1.0,  0.0,                 // fl-x, fl-y
        -1.0,  0.0,                 // rl-x, rl-y
         0.0, -0.25806452,         // rr-x, rr-y
         0.0,  0.25806452          // fr-x, fr-y
    };

    hddc2b_drv_vel_gnd_to_pvt(
            NUM_DRV,
            wheel_distance,
            castor_offset,
            xd_whl,
            xd_drv);

    for (int i = 0; i < NUM_DRV * NUM_DRV_COORD; i++) {
        ck_assert_dbl_eq(xd_drv[i], res[i]);
    }
}
END_TEST


START_TEST(test_hddc2b_drv_vel_pvt_to_gnd)
{
    double xd_drv[NUM_DRV * NUM_DRV_COORD] = {
         1.0,  0.0,                 // fl-x, fl-y
        -1.0,  0.0,                 // rl-x, rl-y
         0.0, -0.25806452,          // rr-x, rr-y
         0.0,  0.25806452           // fr-x, fr-y
    };
    double xd_whl[NUM_DRV * NUM_GND_COORD];
    double res[NUM_DRV * NUM_GND_COORD] = {
        -1.0,  1.0,                 // fl-r, fl-l
         1.0, -1.0,                 // rl-r, rl-l
         1.0,  1.0,                 // rr-r, rr-l
        -1.0, -1.0                  // fr-r, fr-l
    };

    hddc2b_drv_vel_pvt_to_gnd(
            NUM_DRV,
            wheel_distance,
            castor_offset,
            xd_drv,
            xd_whl);

    for (int i = 0; i < NUM_DRV * NUM_GND_COORD; i++) {
        ck_assert_dbl_eq(xd_whl[i], res[i]);
    }
}
END_TEST


START_TEST(test_hddc2b_drv_vel_algn_dst)
{
    double xd_drv[NUM_DRV * NUM_DRV_COORD] = {
         1.0,  0.0,                 // fl: aligned (rolls along +x)
         0.0,  1.0,                 // rl: 90 deg off
         1.0,  1.0,                 // rr: 45 deg off
        -1.0,  0.0                  // fr: 180 deg off
    };
    double dst[NUM_DRV] = { 0.0, 0.0, 0.0, 0.0 };
    double res[NUM_DRV] = {
        0.0,
        M_PI_2,
        M_PI_4,
        M_PI
    };

    hddc2b_drv_vel_algn_dst(NUM_DRV, xd_drv, dst, 1);

    for (int i = 0; i < NUM_DRV; i++) {
        ck_assert_dbl_eq(dst[i], res[i]);
    }
}
END_TEST


START_TEST(test_power_must_be_equal_in_both_spaces)
{
    double xd_whl[NUM_DRV * NUM_GND_COORD] = {
         1.45187253, 25.25703391,   // fl-r, fl-l
        94.49104517,  5.58535653,   // fl-r, fl-l
        59.33541863, 35.61239572,   // rr-r, rr-l
        26.92297075,  6.2835019     // fr-r, fr-l
    };
    double f_drv[NUM_DRV * NUM_DRV_COORD] = {
        70.95956555, 36.06250027,   // fl-x, fl-y
        77.00068869, 82.98443704,   // rl-x, rl-y
        87.41332935, 50.29720317,   // rr-x, rr-y
        23.6478908 , 75.43688388    // fr-x, fr-y
    };
    double xd_drv[NUM_DRV * NUM_DRV_COORD];
    double f_whl[NUM_DRV * NUM_GND_COORD];
    double p_whl[NUM_DRV];
    double p_drv[NUM_DRV];

    hddc2b_drv_vel_gnd_to_pvt(
            NUM_DRV,
            wheel_distance,
            castor_offset,
            xd_whl,
            xd_drv);

    hddc2b_drv_frc_pvt_to_gnd(
            NUM_DRV,
            wheel_distance,
            castor_offset,
            f_drv,
            f_whl);

    for (int i = 0; i < NUM_DRV; i++) {
        p_whl[i] = 0.0;
        p_drv[i] = 0.0;
        for (int j = 0; j < NUM_DRV_COORD; j++) {
            p_whl[i] += xd_whl[i * NUM_DRV_COORD + j] * f_whl[i * NUM_DRV_COORD + j];
            p_drv[i] += xd_drv[i * NUM_DRV_COORD + j] * f_drv[i * NUM_DRV_COORD + j];
        }
        ck_assert_dbl_eq(p_whl[i], p_drv[i]);
    }
}
END_TEST


START_TEST(test_hddc2b_drv_vel_scrb_dst_platform_at_rest)
{
    // While the platform does not rotate, the castor does not have to turn
    // along with it and the scrub angle reduces to the pivot alignment
    // distance of @ref hddc2b_drv_vel_algn_dst.
    double xd_drv[NUM_DRV * NUM_DRV_COORD] = {
         1.0,  0.0,                 // fl: aligned (rolls along +x)
         0.0,  1.0,                 // rl: 90 deg off
         1.0,  1.0,                 // rr: 45 deg off
        -1.0,  0.0                  // fr: 180 deg off
    };
    double dst[NUM_DRV] = { 0.0, 0.0, 0.0, 0.0 };
    double res[NUM_DRV] = {
        0.0,
        M_PI_2,
        M_PI_4,
        M_PI
    };

    hddc2b_drv_vel_scrb_dst(NUM_DRV, castor_offset, 0.0, xd_drv, dst, 1);

    for (int i = 0; i < NUM_DRV; i++) {
        ck_assert_dbl_eq(dst[i], res[i]);
    }
}
END_TEST


START_TEST(test_hddc2b_drv_vel_scrb_dst_platform_rotating)
{
    // The angle is measured at the wheel axle's centre for a castor that turns
    // with the platform: at "omega_pltf" that centre moves transversely at
    // "l omega_p", so a drive whose pivot provides exactly that scrubs not at
    // all, however fast the platform spins.
    const double omega_pltf = 2.0;  // castor offset 0.01 m: l * omega_p = 0.02
    double xd_drv[NUM_DRV * NUM_DRV_COORD] = {
         1.0,  0.02,                // fl: turns along, no scrubbing
         0.02, 0.04,                // rl: 45 deg off
         0.0,  0.52,                // rr: 90 deg off
        -1.0,  0.02                 // fr: 180 deg off
    };
    double dst[NUM_DRV] = { 0.0, 0.0, 0.0, 0.0 };
    double res[NUM_DRV] = {
        0.0,
        M_PI_4,
        M_PI_2,
        M_PI
    };

    hddc2b_drv_vel_scrb_dst(NUM_DRV, castor_offset, omega_pltf, xd_drv, dst, 1);

    for (int i = 0; i < NUM_DRV; i++) {
        ck_assert_dbl_eq(dst[i], res[i]);
    }
}
END_TEST


START_TEST(test_hddc2b_drv_vel_scrb_dst_degenerate)
{
    // A drive that rolls exactly backwards is steered counter-clockwise for
    // either sign of the transverse zero, and below the velocity tolerance
    // there is no direction to align to at all.
    double xd_drv[NUM_DRV * NUM_DRV_COORD] = {
        -1.0,  0.0,                 // fl: backwards, transverse +0
        -1.0, -0.0,                 // rl: backwards, transverse -0
         0.0,  0.0,                 // rr: at rest
         1e-6, 0.0                  // fr: below the velocity tolerance
    };
    double dst[NUM_DRV] = { 0.0, 0.0, 0.0, 0.0 };
    double res[NUM_DRV] = {
        M_PI,
        M_PI,
        0.0,
        0.0
    };

    hddc2b_drv_vel_scrb_dst(NUM_DRV, castor_offset, 0.0, xd_drv, dst, 1);

    for (int i = 0; i < NUM_DRV; i++) {
        ck_assert_dbl_eq(dst[i], res[i]);
    }
}
END_TEST


START_TEST(test_hddc2b_drv_vel_algn_ref)
{
    // The transverse velocity that removes the scrub angle within "tau",
    // saturated at "qd_max"; the castor offset turns the rate into a velocity.
    const double tau    = 0.1;
    const double qd_max = 5.0;
    double dst[NUM_DRV] = {
        0.0,                        // fl: aligned, no pivot motion needed
        0.5,                        // rl: 0.5 / 0.1 = 5 rad/s
       -0.5,                        // rr: -5 rad/s
        M_PI                        // fr: 31.4 rad/s, saturated at 5 rad/s
    };
    double xd_ref[NUM_DRV];
    double res[NUM_DRV] = {         // l * qd
        0.0,
        0.05,
       -0.05,
        0.05
    };

    hddc2b_drv_vel_algn_ref(NUM_DRV, castor_offset, tau, qd_max, 0.0,
            dst, 1, xd_ref, 1);

    for (int i = 0; i < NUM_DRV; i++) {
        ck_assert_dbl_eq(xd_ref[i], res[i]);
    }
}
END_TEST


START_TEST(test_hddc2b_drv_vel_algn_ref_strided)
{
    // Written into the transverse row of a drive velocity matrix, leaving the
    // longitudinal row alone. An aligned drive still turns with the platform,
    // so its reference is the castor's share of the platform rotation.
    const double omega_pltf = 3.0;
    double dst[NUM_DRV] = { 0.0, 0.0, 0.0, 0.0 };
    double xd_ref[NUM_DRV * NUM_DRV_COORD] = {
        1.0, 0.0,                   // fl-x, fl-y
        2.0, 0.0,                   // rl-x, rl-y
        3.0, 0.0,                   // rr-x, rr-y
        4.0, 0.0                    // fr-x, fr-y
    };
    double res[NUM_DRV * NUM_DRV_COORD] = {
        1.0, 0.03,                  // l * omega_pltf
        2.0, 0.03,
        3.0, 0.03,
        4.0, 0.03
    };

    hddc2b_drv_vel_algn_ref(NUM_DRV, castor_offset, 0.1, 5.0, omega_pltf,
            dst, 1, &xd_ref[1], NUM_DRV_COORD);

    for (int i = 0; i < NUM_DRV * NUM_DRV_COORD; i++) {
        ck_assert_dbl_eq(xd_ref[i], res[i]);
    }
}
END_TEST


TCase *hddc2b_drive_test(void)
{
    TCase *tc = tcase_create("drive");

    tcase_add_test(tc, test_hddc2b_drv_frc_gnd_to_pvt);
    tcase_add_test(tc, test_hddc2b_drv_frc_pvt_to_gnd);
    tcase_add_test(tc, test_hddc2b_drv_vel_gnd_to_pvt);
    tcase_add_test(tc, test_hddc2b_drv_vel_pvt_to_gnd);
    tcase_add_test(tc, test_hddc2b_drv_vel_algn_dst);
    tcase_add_test(tc, test_hddc2b_drv_vel_scrb_dst_platform_at_rest);
    tcase_add_test(tc, test_hddc2b_drv_vel_scrb_dst_platform_rotating);
    tcase_add_test(tc, test_hddc2b_drv_vel_scrb_dst_degenerate);
    tcase_add_test(tc, test_hddc2b_drv_vel_algn_ref);
    tcase_add_test(tc, test_hddc2b_drv_vel_algn_ref_strided);
    tcase_add_test(tc, test_power_must_be_equal_in_both_spaces);

    return tc;
}
