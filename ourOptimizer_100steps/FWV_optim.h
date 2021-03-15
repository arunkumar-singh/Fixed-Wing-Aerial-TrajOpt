#pragma once
#include "eigen-3.3.8/Eigen/Dense"
//#include<Eigen/Dense>
using namespace Eigen;

namespace optim
{
    struct three_var
    {
        ArrayXXf a, b, c;
    };
    struct two_var
    {
        ArrayXXf a, b;
    };
    struct six_var
    {
        ArrayXXf a, b, c, d, e, f;
    };
    struct seven_var
    {
        ArrayXXf a, b, c, d, e, f, g;
    };
    struct nine_var
    {
        ArrayXXf a, b, c, d, e, f, g, h, i;
    };
    struct eleven_var
    {
        ArrayXXf a, b, c, d, e, f, g, h, i, j, k;
    };
    ArrayXXf clip2(ArrayXXf min, ArrayXXf max, ArrayXXf arr);
    ArrayXXf clip(float min, float max, ArrayXXf arr);
    ArrayXXf diff(ArrayXXf arr);
    ArrayXXf maximum(float val, ArrayXXf arr2);
    ArrayXXf minimum(float val, ArrayXXf arr2);
    ArrayXXf linspace(float t_init, float t_end, float steps);
    float binomialCoeff(float n, float k);
    ArrayXXf arctan2(ArrayXXf arr1, ArrayXXf arr2);
    ArrayXXf stack(ArrayXXf arr1, ArrayXXf arr2, char ch);
    ArrayXXf cumsum(ArrayXXf arr1, ArrayXXf arr2);

    three_var bernstein_coeff_order10_new(float n, float tmin, float tmax, ArrayXXf t_actual, int num);
    nine_var init_guess_compute(ArrayXXf P, ArrayXXf Pdot, ArrayXXf Pddot, int num_obs, float x_init, float y_init, float z_init, float vx_init, float vy_init, float vz_init,
        float x_d, float y_d, float z_d, ArrayXXf x_obs, ArrayXXf y_obs, ArrayXXf z_obs, ArrayXXf a_obs, ArrayXXf b_obs, ArrayXXf c_obs);

    six_var compute_initial_guess(int num_obs, int num, int nvar, ArrayXXf P, ArrayXXf Pdot, ArrayXXf Pddot, ArrayXXf A_eq, ArrayXXf bx_eq, ArrayXXf by_eq, 
                               ArrayXXf bz_eq, ArrayXXf cost_mat_inv, ArrayXXf A_obs, ArrayXXf alpha_obs, ArrayXXf beta_obs, float rho_obs, ArrayXXf x_obs, 
                               ArrayXXf y_obs, ArrayXXf z_obs, ArrayXXf a_obs, ArrayXXf b_obs, ArrayXXf c_obs, ArrayXXf d_obs, ArrayXXf lamda_obs_x, 
                               ArrayXXf lamda_obs_y, ArrayXXf lamda_obs_z);

    ArrayXXf compute_cpsi_fixed(float psi_init, float psidot_init, float psiddot_init, ArrayXXf P, ArrayXXf Pdot, ArrayXXf Pddot);
    
    two_var compute_smoothness_cost_psi(int num, int nvar, float weight_psi, ArrayXXf cpsi_fixed, ArrayXXf P, ArrayXXf Pdot, ArrayXXf Pddot);
    
    seven_var compute_psi(ArrayXXf P, ArrayXXf Pdot, ArrayXXf Pddot, int nvar, int num, ArrayXXf cost_smoothness_psi, ArrayXXf lincost_smoothness_psi, ArrayXXf v, ArrayXXf psi_temp,
        ArrayXXf cpsi_fixed, ArrayXXf lamda_psi, float rho_psi, ArrayXXf lamda_psidot_ineq, float rho_psidot_ineq, ArrayXXf s_psidot_ineq, float phi_max, float psi_term, float weight_psi_term);

    six_var compute_x(int num_obs, ArrayXXf P, ArrayXXf Pdot, ArrayXXf Pddot, ArrayXXf cost_smoothness, ArrayXXf A_eq, ArrayXXf b_eq_x,
        ArrayXXf b_eq_y, ArrayXXf b_eq_z, ArrayXXf A_nonhol, ArrayXXf psi, ArrayXXf gamma_, ArrayXXf v, float rho_nonhol, ArrayXXf lamda_nonhol_x,
        ArrayXXf lamda_nonhol_y, ArrayXXf lamda_nonhol_z, ArrayXXf A_goal, ArrayXXf b_goal_x, ArrayXXf b_goal_y, ArrayXXf b_goal_z, float weight_goal,
        ArrayXXf d_obs, ArrayXXf alpha_obs, ArrayXXf beta_obs, ArrayXXf a_obs, ArrayXXf b_obs, ArrayXXf c_obs, ArrayXXf x_obs, ArrayXXf y_obs, ArrayXXf z_obs,
        ArrayXXf A_obs, float rho_obs);

}
