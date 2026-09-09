#include <hddc2b/functions/platform.h>
#include <hddc2b/functions/solver.h>
#include <hddc2b/functions/drive.h>
#include <hddc2b/functions/wheel.h>
#include <solver.h>
#include <assert.h>
#include <math.h>


void hddc2b_example_frc(
        int num_drv,
        double eps,
        const double *g,
        const double *w_pltf,
        const double *f_pltf,
        const double *w_drv,
        const double *f_drv_ref,
        double *f_drv)
{
    assert(num_drv >= 0);

    const int NUM_DRV_COORD  = 2;
    const int NUM_PLTF_COORD = 3;
    const int NUM_G_COORD    = NUM_PLTF_COORD * NUM_DRV_COORD;


    double w_pltf_sqrt[NUM_PLTF_COORD * NUM_PLTF_COORD];
    hddc2b_pltf_frc_w_pltf_sqrt(w_pltf, w_pltf_sqrt);
    double w_drv_inv_sqrt[num_drv * NUM_DRV_COORD * NUM_DRV_COORD];
    hddc2b_pltf_frc_w_drv_inv_sqrt(num_drv, w_drv, w_drv_inv_sqrt);
    double g2[num_drv * NUM_G_COORD];
    double f_pltf2[NUM_PLTF_COORD];
    hddc2b_pltf_frc_sing_wgh(num_drv, g, f_pltf, w_pltf_sqrt, g2, f_pltf2);
    double f_pltf3[NUM_PLTF_COORD];
    hddc2b_pltf_frc_redu_ref_init(num_drv, g2, f_pltf2, f_drv_ref, f_pltf3);
    double g3[num_drv * NUM_G_COORD];
    hddc2b_pltf_frc_redu_wgh_init(num_drv, g2, w_drv_inv_sqrt, g3);
    double u[NUM_PLTF_COORD * NUM_PLTF_COORD];
    double s[NUM_PLTF_COORD];
    double vt[num_drv * NUM_G_COORD];
    hddc2b_pltf_dcmp(num_drv, g3, u, s, vt);

    double s_inv[NUM_PLTF_COORD];
    hddc2b_pltf_pinv(num_drv, eps, s, s_inv);
    double f_drv2[num_drv * NUM_DRV_COORD];
    hddc2b_pltf_frc_slv(num_drv, u, s_inv, vt, f_pltf3, f_drv2);
    double f_drv3[num_drv * NUM_DRV_COORD];
    hddc2b_pltf_frc_redu_wgh_fini(num_drv, f_drv2, w_drv_inv_sqrt, f_drv3);
    hddc2b_pltf_frc_redu_ref_fini(num_drv, f_drv_ref, f_drv3, f_drv);
}


void hddc2b_example_vel(
        int num_drv,
        double eps,
        const double *g,
        const double *w_drv_sqrt,
        const double *xd_drv,
        const double *w_pltf_inv_sqrt,
        double *xd_pltf)
{
    assert(num_drv >= 0);

    const int NUM_DRV_COORD  = 2;
    const int NUM_PLTF_COORD = 3;
    const int NUM_G_COORD    = NUM_PLTF_COORD * NUM_DRV_COORD;


    double g2[num_drv * NUM_G_COORD];
    double xd_drv2[num_drv * NUM_DRV_COORD];
    hddc2b_pltf_vel_sing_wgh(num_drv, g, xd_drv, w_drv_sqrt, g2, xd_drv2);
    double g3[num_drv * NUM_G_COORD];
    hddc2b_pltf_vel_redu_wgh_init(num_drv, g2, w_pltf_inv_sqrt, g3);
    double u[NUM_PLTF_COORD * NUM_PLTF_COORD];
    double s[NUM_PLTF_COORD];
    double vt[num_drv * NUM_G_COORD];
    hddc2b_pltf_dcmp(num_drv, g3, u, s, vt);

    double s_inv[NUM_PLTF_COORD];
    hddc2b_pltf_pinv(num_drv, eps, s, s_inv);
    double xd_pltf2[NUM_PLTF_COORD];
    hddc2b_pltf_vel_slv(num_drv, u, s_inv, vt, xd_drv2, xd_pltf2);
    hddc2b_pltf_vel_redu_wgh_fini(num_drv, xd_pltf2, w_pltf_inv_sqrt, xd_pltf);
}


void ex_frc_pltf_to_pvt_redu_sing_pinv(
        int num_drv,
        double eps,
        const double *g,
        const double *w_pltf_sqrt,
        const double *f_pltf,
        const double *w_drv_inv_sqrt,
        double *f_drv)
{
    assert(num_drv >= 0);

    const int NUM_DRV_COORD  = 2;
    const int NUM_PLTF_COORD = 3;
    const int NUM_G_COORD    = NUM_PLTF_COORD * NUM_DRV_COORD;


    double g2[num_drv * NUM_G_COORD];
    double f_pltf2[NUM_PLTF_COORD];
    hddc2b_pltf_frc_sing_wgh(num_drv, g, f_pltf, w_pltf_sqrt, g2, f_pltf2);
    double g3[num_drv * NUM_G_COORD];
    hddc2b_pltf_frc_redu_wgh_init(num_drv, g2, w_drv_inv_sqrt, g3);
    double u[NUM_PLTF_COORD * NUM_PLTF_COORD];
    double s[NUM_PLTF_COORD];
    double vt[num_drv * NUM_G_COORD];
    hddc2b_pltf_dcmp(num_drv, g3, u, s, vt);

    double s_inv[NUM_PLTF_COORD];
    hddc2b_pltf_pinv(num_drv, eps, s, s_inv);
    double f_drv2[num_drv * NUM_DRV_COORD];
    hddc2b_pltf_frc_slv(num_drv, u, s_inv, vt, f_pltf2, f_drv2);
    hddc2b_pltf_frc_redu_wgh_fini(num_drv, f_drv2, w_drv_inv_sqrt, f_drv);
}


void ex_frc_pltf_to_pvt_redu_sing_ref_pinv(
        int num_drv,
        double eps,
        const double *g,
        const double *w_pltf_sqrt,
        const double *f_pltf,
        const double *w_drv_inv_sqrt,
        const double *f_drv_ref,
        double *f_drv)
{
    assert(num_drv >= 0);

    const int NUM_DRV_COORD  = 2;
    const int NUM_PLTF_COORD = 3;
    const int NUM_G_COORD    = NUM_PLTF_COORD * NUM_DRV_COORD;


    double g2[num_drv * NUM_G_COORD];
    double f_pltf2[NUM_PLTF_COORD];
    hddc2b_pltf_frc_sing_wgh(num_drv, g, f_pltf, w_pltf_sqrt, g2, f_pltf2);
    double f_pltf3[NUM_PLTF_COORD];
    hddc2b_pltf_frc_redu_ref_init(num_drv, g2, f_pltf2, f_drv_ref, f_pltf3);
    double g3[num_drv * NUM_G_COORD];
    hddc2b_pltf_frc_redu_wgh_init(num_drv, g2, w_drv_inv_sqrt, g3);
    double u[NUM_PLTF_COORD * NUM_PLTF_COORD];
    double s[NUM_PLTF_COORD];
    double vt[num_drv * NUM_G_COORD];
    hddc2b_pltf_dcmp(num_drv, g3, u, s, vt);

    double s_inv[NUM_PLTF_COORD];
    hddc2b_pltf_pinv(num_drv, eps, s, s_inv);
    double f_drv2[num_drv * NUM_DRV_COORD];
    hddc2b_pltf_frc_slv(num_drv, u, s_inv, vt, f_pltf3, f_drv2);
    double f_drv3[num_drv * NUM_DRV_COORD];
    hddc2b_pltf_frc_redu_wgh_fini(num_drv, f_drv2, w_drv_inv_sqrt, f_drv3);
    hddc2b_pltf_frc_redu_ref_fini(num_drv, f_drv_ref, f_drv3, f_drv);
}


void ex_frc_pltf_to_pvt_sing_dls(
        int num_drv,
        double eps,
        double lambda,
        const double *g,
        const double *w_pltf_sqrt,
        const double *f_pltf,
        double *f_drv)
{
    assert(num_drv >= 0);

    const int NUM_DRV_COORD  = 2;
    const int NUM_PLTF_COORD = 3;
    const int NUM_G_COORD    = NUM_PLTF_COORD * NUM_DRV_COORD;


    double g2[num_drv * NUM_G_COORD];
    double f_pltf2[NUM_PLTF_COORD];
    hddc2b_pltf_frc_sing_wgh(num_drv, g, f_pltf, w_pltf_sqrt, g2, f_pltf2);
    double u[NUM_PLTF_COORD * NUM_PLTF_COORD];
    double s[NUM_PLTF_COORD];
    double vt[num_drv * NUM_G_COORD];
    hddc2b_pltf_dcmp(num_drv, g2, u, s, vt);

    double s_inv[NUM_PLTF_COORD];
    hddc2b_pltf_dmp(num_drv, eps, lambda, s, s_inv);
    hddc2b_pltf_frc_slv(num_drv, u, s_inv, vt, f_pltf2, f_drv);
}


void ex_frc_pltf_to_pvt_redu_ref_pinv(
        int num_drv,
        double eps,
        const double *g,
        const double *f_pltf,
        const double *w_drv_inv_sqrt,
        const double *f_drv_ref,
        double *f_drv)
{
    assert(num_drv >= 0);

    const int NUM_DRV_COORD  = 2;
    const int NUM_PLTF_COORD = 3;
    const int NUM_G_COORD    = NUM_PLTF_COORD * NUM_DRV_COORD;


    double f_pltf3[NUM_PLTF_COORD];
    hddc2b_pltf_frc_redu_ref_init(num_drv, g, f_pltf, f_drv_ref, f_pltf3);
    double g3[num_drv * NUM_G_COORD];
    hddc2b_pltf_frc_redu_wgh_init(num_drv, g, w_drv_inv_sqrt, g3);
    double u[NUM_PLTF_COORD * NUM_PLTF_COORD];
    double s[NUM_PLTF_COORD];
    double vt[num_drv * NUM_G_COORD];
    hddc2b_pltf_dcmp(num_drv, g3, u, s, vt);

    double s_inv[NUM_PLTF_COORD];
    hddc2b_pltf_pinv(num_drv, eps, s, s_inv);
    double f_drv2[num_drv * NUM_DRV_COORD];
    hddc2b_pltf_frc_slv(num_drv, u, s_inv, vt, f_pltf3, f_drv2);
    double f_drv3[num_drv * NUM_DRV_COORD];
    hddc2b_pltf_frc_redu_wgh_fini(num_drv, f_drv2, w_drv_inv_sqrt, f_drv3);
    hddc2b_pltf_frc_redu_ref_fini(num_drv, f_drv_ref, f_drv3, f_drv);
}


void ex_vel_pvt_to_pltf_sing_dls(
        int num_drv,
        double eps,
        double lambda,
        const double *g,
        const double *xd_drv,
        const double *w_pltf_inv_sqrt,
        double *xd_pltf)
{
    assert(num_drv >= 0);

    const int NUM_DRV_COORD  = 2;
    const int NUM_PLTF_COORD = 3;
    const int NUM_G_COORD    = NUM_PLTF_COORD * NUM_DRV_COORD;


    double g3[num_drv * NUM_G_COORD];
    hddc2b_pltf_vel_redu_wgh_init(num_drv, g, w_pltf_inv_sqrt, g3);
    double u[NUM_PLTF_COORD * NUM_PLTF_COORD];
    double s[NUM_PLTF_COORD];
    double vt[num_drv * NUM_G_COORD];
    hddc2b_pltf_dcmp(num_drv, g3, u, s, vt);

    double s_inv[NUM_PLTF_COORD];
    hddc2b_pltf_dmp(num_drv, eps, lambda, s, s_inv);
    double xd_pltf2[NUM_PLTF_COORD];
    hddc2b_pltf_vel_slv(num_drv, u, s_inv, vt, xd_drv, xd_pltf2);
    hddc2b_pltf_vel_redu_wgh_fini(num_drv, xd_pltf2, w_pltf_inv_sqrt, xd_pltf);
}


void ex_vel_pltf_to_pvt(
        int num_drv,
        const double *g,
        const double *xd_pltf,
        double *xd_drv)
{
    assert(num_drv >= 0);

    hddc2b_pltf_vel_pltf_to_pvt(num_drv, g, xd_pltf, xd_drv);
}


void hddc2b_example_vel_dist(
        int num_drv,
        const double *g,
        const double *whl_dia,
        const double *whl_dst,
        const double *cstr_off,
        const double *w_pltf,
        const double *w_algn,
        double tau,
        double qd_max,
        double omega_max,
        const double *xd_pltf,
        double *xd_pltf_eff,
        double *xd_drv,
        double *omega_hub)
{
    assert(num_drv >= 0);

    const int NUM_DRV_COORD = 2;
    double xd_whl[num_drv * NUM_DRV_COORD];
    double xd_drv_prim[num_drv * NUM_DRV_COORD];
    double dst[num_drv];
    double spd[num_drv];
    double xd_drv_ref[num_drv * NUM_DRV_COORD];

    hddc2b_pltf_vel_pltf_to_pvt(num_drv, g, xd_pltf, xd_drv_prim);
    hddc2b_drv_vel_scrb_dst(num_drv, cstr_off, xd_pltf[2], xd_drv_prim,
            dst, 1, spd, 1);

    // The castor alignment task only constrains the drives' transverse
    // velocity, hence the longitudinal reference remains unused
    for (int i = 0; i < num_drv * NUM_DRV_COORD; i++) {
        xd_drv_ref[i] = 0.0;
    }
    hddc2b_drv_vel_algn_ref(num_drv, cstr_off, tau, qd_max, xd_pltf[2],
            dst, 1, spd, 1, &xd_drv_ref[1], NUM_DRV_COORD);

    hddc2b_pltf_vel_algn_rlx(num_drv, g, w_pltf, w_algn, xd_pltf,
            xd_drv_ref, xd_pltf_eff);
    hddc2b_pltf_vel_pltf_to_pvt(num_drv, g, xd_pltf_eff, xd_drv);

    hddc2b_drv_vel_pvt_to_gnd(num_drv, whl_dst, cstr_off, xd_drv, xd_whl);
    hddc2b_whl_vel_gnd_to_hub(num_drv, whl_dia, xd_whl, omega_hub);
    hddc2b_pltf_vel_hub_lim(num_drv, omega_max,
            xd_pltf_eff, xd_drv, omega_hub,
            xd_pltf_eff, xd_drv, omega_hub);
}
