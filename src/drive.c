// SPDX-License-Identifier: LGPL-3.0
#include <hddc2b/functions/drive.h>
#include <config.h>

#include <math.h>
#include <assert.h>


// Tolerance for when to consider the vector length close to zero
static const double EPS = 0.00001;

// Tolerance for when to consider an angle to lie on atan2's -pi branch
static const double EPS_ANG = 0.000000001;


void hddc2b_drv_frc_gnd_to_pvt(
        int num_drv,
        const double *whl_dst,
        const double *cstr_off,
        const double *f_whl,
        double *f_drv)
{
    assert(num_drv >= 0);
    assert(whl_dst);
    assert(cstr_off);
    assert(f_whl);
    assert(f_drv);

    for (int i = 0; i < num_drv; i++) {
        int left  = i * 2 + OFFSET_LEFT;
        int right = i * 2 + OFFSET_RIGHT;
        int x = 0 + i * 2;  // longitudinal force
        int y = 1 + i * 2;  // transverse force
        double r = whl_dst[i];
        double l = cstr_off[i];

        assert(fabs(l) > 0.0);

        f_drv[x] =      1.0     * (f_whl[left] - f_whl[right]);
        f_drv[y] = -0.5 * r / l * (f_whl[left] + f_whl[right]);
    }
}


void hddc2b_drv_frc_pvt_to_gnd(
        int num_drv,
        const double *whl_dst,
        const double *cstr_off,
        const double *f_drv,
        double *f_whl)
{
    assert(num_drv >= 0);
    assert(whl_dst);
    assert(cstr_off);
    assert(f_whl);
    assert(f_drv);

    for (int i = 0; i < num_drv; i++) {
        int left  = i * 2 + OFFSET_LEFT;
        int right = i * 2 + OFFSET_RIGHT;
        int x = 0 + i * 2;  // longitudinal force
        int y = 1 + i * 2;  // transverse force
        double r = whl_dst[i];
        double l = cstr_off[i];

        assert(fabs(r) > 0.0);

        f_whl[right] = -0.5 * f_drv[x] - l / r * f_drv[y];
        f_whl[left ] =  0.5 * f_drv[x] - l / r * f_drv[y];
    }
}


void hddc2b_drv_vel_gnd_to_pvt(
        int num_drv,
        const double *whl_dst,
        const double *cstr_off,
        const double *xd_whl,
        double *xd_drv)
{
    assert(num_drv >= 0);
    assert(whl_dst);
    assert(cstr_off);
    assert(xd_whl);
    assert(xd_drv);

    for (int i = 0; i < num_drv; i++) {
        int left  = i * 2 + OFFSET_LEFT;
        int right = i * 2 + OFFSET_RIGHT;
        int x = 0 + i * 2;  // longitudinal force
        int y = 1 + i * 2;  // transverse force
        double r = whl_dst[i];
        double l = cstr_off[i];

        assert(fabs(l) > 0.0);

        xd_drv[x] =   0.5  * (xd_whl[left] - xd_whl[right]);
        xd_drv[y] = -l / r * (xd_whl[left] + xd_whl[right]);
    }
}


void hddc2b_drv_vel_pvt_to_gnd(
        int num_drv,
        const double *whl_dst,
        const double *cstr_off,
        const double *xd_drv,
        double *xd_whl)
{
    assert(num_drv >= 0);
    assert(whl_dst);
    assert(cstr_off);
    assert(xd_drv);
    assert(xd_whl);

    for (int i = 0; i < num_drv; i++) {
        int left  = i * 2 + OFFSET_LEFT;
        int right = i * 2 + OFFSET_RIGHT;
        int x = 0 + i * 2;  // longitudinal velocity
        int y = 1 + i * 2;  // transverse velocity
        double r = whl_dst[i];
        double l = cstr_off[i];

        assert(fabs(l) > 0.0);

        xd_whl[right] = -1.0 * xd_drv[x] - 0.5 * r / l * xd_drv[y];
        xd_whl[left ] =  1.0 * xd_drv[x] - 0.5 * r / l * xd_drv[y];
    }
}


void hddc2b_drv_vel_algn_dst(
        int num_drv,
        const double *xd_drv,
        double *dst,
        int inc_dst)
{
    assert(num_drv >= 0);
    assert(xd_drv);
    assert(dst);
    assert(inc_dst >= 0);

    for (int i = 0; i < num_drv; i++) {
        int x = 0 + i * 2;  // longitudinal velocity
        int y = 1 + i * 2;  // transverse velocity

        // Signed angle between the drive's rolling direction (the pivot frame's
        // x-axis) and the commanded attachment velocity. Rotating the pivot by
        // this angle makes the drive roll without lateral scrubbing
        // (transverse velocity zero).
        dst[i * inc_dst] = atan2(xd_drv[y], xd_drv[x]);
    }
}


void hddc2b_drv_vel_scrb_dst(
        int num_drv,
        const double *cstr_off,
        double omega_pltf,
        const double *xd_drv,
        double *dst,
        int inc_dst)
{
    assert(num_drv >= 0);
    assert(cstr_off);
    assert(xd_drv);
    assert(dst);
    assert(inc_dst >= 0);

    for (int i = 0; i < num_drv; i++) {
        int x = 0 + i * 2;  // longitudinal velocity
        int y = 1 + i * 2;  // transverse velocity
        double l = cstr_off[i];

        // Velocity of the wheel axle's centre for a castor that turns with the
        // platform, i.e. that does not move with respect to the platform
        double vx = xd_drv[x];
        double vy = xd_drv[y] - l * omega_pltf;

        double d = 0.0;
        if (hypot(vx, vy) > EPS) {
            d = atan2(vy, vx);

            // For a drive that rolls exactly backwards atan2 returns either
            // "-pi" or "+pi", depending on the sign of the zero. Always
            // choose "+pi" so that such drives are consistently steered in
            // counter-clockwise direction.
            if (d < -M_PI + EPS_ANG) {
                d = M_PI;
            }
        }

        dst[i * inc_dst] = d;
    }
}


void hddc2b_drv_vel_algn_ref(
        int num_drv,
        const double *cstr_off,
        double tau,
        double qd_max,
        double omega_pltf,
        const double *dst,
        int inc_dst,
        double *xd_ref,
        int inc_ref)
{
    assert(num_drv >= 0);
    assert(cstr_off);
    assert(tau > 0.0);
    assert(qd_max >= 0.0);
    assert(dst);
    assert(inc_dst >= 0);
    assert(xd_ref);
    assert(inc_ref >= 0);

    for (int i = 0; i < num_drv; i++) {
        double l = cstr_off[i];

        // Rate at which the castor turns with respect to the platform to
        // remove the scrub angle within the time constant, saturated at the
        // maximum castor rate
        double qd = dst[i * inc_dst] / tau;
        if (qd > qd_max) {
            qd = qd_max;
        }
        if (qd < -qd_max) {
            qd = -qd_max;
        }

        // A castor rotates at "\dot{q}" in the world when its pivot point
        // moves transversely at "l \dot{q}"
        xd_ref[i * inc_ref] = l * (qd + omega_pltf);
    }
}
