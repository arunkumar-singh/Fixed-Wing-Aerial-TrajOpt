#include <iostream>
#include "FWV_optim.h"
using namespace std;

namespace optim
{

	ArrayXXf clip2(ArrayXXf min, ArrayXXf max, ArrayXXf arr)
	{
	    for (int k = 0; k < arr.cols() * arr.rows(); k++)
	    {
		if (arr(k) > max(k))
		    arr(k) = max(k);
		if (arr(k) < min(k))
		    arr(k) = min(k);
	    }
	    return arr;
	}
	ArrayXXf clip(float min, float max, ArrayXXf arr)
	{
	    for (int k = 0; k < arr.cols() * arr.rows(); k++)
	    {
		if (arr(k) > max)
		    arr(k) = max;
		if (arr(k) < min)
		    arr(k) = min;
	    }
	    return arr;
	}
	ArrayXXf diff(ArrayXXf arr)
	{
	    ArrayXXf temp(arr.rows() - 1, 1);
	    for (int i = 0; i < arr.rows() - 1; i++)
	    {
		temp(i) = arr(i + 1) - arr(i);
	    }
	    return temp;
	}
	ArrayXXf maximum(float val, ArrayXXf arr2)
	{
	    ArrayXXf temp(arr2.rows(), arr2.cols());
	    temp = val;

	    int k = 0;
	    for (int i = 0; i < arr2.cols(); i++)
	    {
		for (int j = 0; j < arr2.rows(); j++)
		{
		    if (arr2(k) > val)
		        temp(k) = arr2(k);
		    k++;
		}
	    }
	    return temp;
	}
	ArrayXXf minimum(float val, ArrayXXf arr2)
	{
	    ArrayXXf temp(arr2.rows(), arr2.cols());
	    temp = val;

	    int k = 0;
	    for (int i = 0; i < arr2.cols(); i++)
	    {
		for (int j = 0; j < arr2.rows(); j++)
		{
		    if (arr2(k) < val)
		        temp(k) = arr2(k);
		    k++;
		}
	    }
	    return temp;
	}
	ArrayXXf linspace(float t_init, float t_end, float steps)
	{
	    ArrayXXf m((int)steps, 1);
	    float delta = (t_end - t_init) / steps;
	    for (float i = 0; i < steps; i++)
	    {
		m(i) = delta * (i + 1);
	    }
	    return m;
	}

	float binomialCoeff(float n, float k)
	{
	    if (k == 0 || k == n)
		return 1;

	    return binomialCoeff(n - 1, k - 1) +
		binomialCoeff(n - 1, k);
	}

	ArrayXXf arctan2(ArrayXXf arr1, ArrayXXf arr2)
	{
	    ArrayXXf temp(arr1.rows(), arr1.cols());

	    int k = 0;
	    for (int i = 0; i < arr1.cols(); i++)
	    {
		for (int j = 0; j < arr1.rows(); j++)
		{
		    temp(k) = atan2(arr1(k), arr2(k));
		    k++;
		}
	    }
	    return temp;
	}

	ArrayXXf stack(ArrayXXf arr1, ArrayXXf arr2, char ch)
	{
	    if (ch == 'v')
	    {
		ArrayXXf temp(arr1.rows() + arr2.rows(), arr1.cols());
		int i;
		for (i = 0; i < arr1.rows(); i++)
		{
		    temp.row(i) = arr1.row(i);
		}

		for (; i < arr1.rows() + arr2.rows(); i++)
		{
		    temp.row(i) = arr2.row(i - arr1.rows());
		}
		return temp;
	    }
	    else
	    {
		ArrayXXf temp(arr1.rows(), arr1.cols() + arr2.cols());
		int i;
		for (i = 0; i < arr1.cols(); i++)
		{
		    temp.col(i) = arr1.col(i);
		}
		for (; i < arr1.cols() + arr2.cols(); i++)
		{
		    temp.col(i) = arr2.col(i - arr1.cols());
		}
		return temp;
	    }

	}

	ArrayXXf cumsum(ArrayXXf arr1, ArrayXXf arr2)
	{

	    float init = arr1(0);

	    for (int i = 0; i < arr1.cols(); i++)
		arr1.col(i) = arr1.col(i) * arr2;

	    int k = 1;
	    for (int j = 0; j < arr1.cols(); j++)
	    {
		for (int i = 1; i < arr1.rows(); i++)
		{
		    arr1(k) = arr1(k) + arr1(k - 1);
		    k++;
		}
		k++;
	    }
	    return arr1;
	}


	//___________________________________________________________Optim Functions____________________________________________________________________
	three_var bernstein_coeff_order10_new(float n, float tmin, float tmax, ArrayXXf t_actual, int num)
	{
	    three_var s;
	    float l = tmax - tmin;
	    ArrayXXf t = (t_actual - tmin) / l;

	    ArrayXXf P(num, (int)n + 1), Pdot(num, (int)n + 1), Pddot(num, (int)n + 1);


	    P.col(0) = binomialCoeff(n, 0) * pow(1 - t, n - 0) * pow(t, 0);

	    P.col(1) = binomialCoeff(n, 1) * pow(1 - t, n - 1) * pow(t, 1);

	    P.col(2) = binomialCoeff(n, 2) * pow(1 - t, n - 2) * pow(t, 2);

	    P.col(3) = binomialCoeff(n, 3) * pow(1 - t, n - 3) * pow(t, 3);

	    P.col(4) = binomialCoeff(n, 4) * pow(1 - t, n - 4) * pow(t, 4);

	    P.col(5) = binomialCoeff(n, 5) * pow(1 - t, n - 5) * pow(t, 5);

	    P.col(6) = binomialCoeff(n, 6) * pow(1 - t, n - 6) * pow(t, 6);

	    P.col(7) = binomialCoeff(n, 7) * pow(1 - t, n - 7) * pow(t, 7);

	    P.col(8) = binomialCoeff(n, 8) * pow(1 - t, n - 8) * pow(t, 8);

	    P.col(9) = binomialCoeff(n, 9) * pow(1 - t, n - 9) * pow(t, 9);

	    P.col(10) = binomialCoeff(n, 10) * pow(1 - t, n - 10) * pow(t, 10);

	    Pdot.col(0) = -10.0 * pow(-t + 1, 9);

	    Pdot.col(1) = -90.0 * t * pow(-t + 1, 8) + 10.0 * pow(-t + 1, 9);

	    Pdot.col(2) = -360.0 * pow(t, 2) * pow(-t + 1, 7) + 90.0 * t * pow(-t + 1, 8);

	    Pdot.col(3) = -840.0 * pow(t, 3) * pow(-t + 1, 6) + 360.0 * pow(t, 2) * pow(-t + 1, 7);

	    Pdot.col(4) = -1260.0 * pow(t, 4) * pow(-t + 1, 5) + 840.0 * pow(t, 3) * pow(-t + 1, 6);

	    Pdot.col(5) = -1260.0 * pow(t, 5) * pow(-t + 1, 4) + 1260.0 * pow(t, 4) * pow(-t + 1, 5);

	    Pdot.col(6) = -840.0 * pow(t, 6) * pow(-t + 1, 3) + 1260.0 * pow(t, 5) * pow(-t + 1, 4);

	    Pdot.col(7) = -360.0 * pow(t, 7) * pow(-t + 1, 2) + 840.0 * pow(t, 6) * pow(-t + 1, 3);

	    Pdot.col(8) = 45.0 * pow(t, 8) * (2 * t - 2) + 360.0 * pow(t, 7) * pow(-t + 1, 2);

	    Pdot.col(9) = -10.0 * pow(t, 9) + 9 * pow(t, 8) * (-10.0 * t + 10.0);

	    Pdot.col(10) = 10.0 * pow(t, 9);


	    Pddot.col(0) = 90.0 * pow(-t + 1, 8.0);

	    Pddot.col(1) = 720.0 * t * pow(-t + 1, 7) - 180.0 * pow(-t + 1, 8);

	    Pddot.col(2) = 2520.0 * pow(t, 2) * pow(-t + 1, 6) - 1440.0 * t * pow(-t + 1, 7) + 90.0 * pow(-t + 1, 8);

	    Pddot.col(3) = 5040.0 * pow(t, 3) * pow(-t + 1, 5) - 5040.0 * pow(t, 2) * pow(-t + 1, 6) + 720.0 * t * pow(-t + 1, 7);

	    Pddot.col(4) = 6300.0 * pow(t, 4) * pow(-t + 1, 4) - 10080.0 * pow(t, 3) * pow(-t + 1, 5) + 2520.0 * pow(t, 2) * pow(-t + 1, 6);

	    Pddot.col(5) = 5040.0 * pow(t, 5) * pow(-t + 1, 3) - 12600.0 * pow(t, 4) * pow(-t + 1, 4) + 5040.0 * pow(t, 3) * pow(-t + 1, 5);

	    Pddot.col(6) = 2520.0 * pow(t, 6) * pow(-t + 1, 2) - 10080.0 * pow(t, 5) * pow(-t + 1, 3) + 6300.0 * pow(t, 4) * pow(-t + 1, 4);

	    Pddot.col(7) = -360.0 * pow(t, 7) * (2 * t - 2) - 5040.0 * pow(t, 6) * pow(-t + 1, 2) + 5040.0 * pow(t, 5) * pow(-t + 1, 3);

	    Pddot.col(8) = 90.0 * pow(t, 8) + 720.0 * pow(t, 7) * (2 * t - 2) + 2520.0 * pow(t, 6) * pow(-t + 1, 2);

	    Pddot.col(9) = -180.0 * pow(t, 8) + 72 * pow(t, 7) * (-10.0 * t + 10.0);

	    Pddot.col(10) = 90.0 * pow(t, 8);

	    s.a = P;
	    s.b = Pdot / l;
	    s.c = Pddot / (l * l);

	    return s;
	}

	six_var compute_initial_guess(int num_obs, int num, int nvar, ArrayXXf P, ArrayXXf Pdot, ArrayXXf Pddot, ArrayXXf A_eq, ArrayXXf bx_eq, ArrayXXf by_eq,
	    ArrayXXf bz_eq, ArrayXXf cost_mat_inv, ArrayXXf A_obs, ArrayXXf alpha_obs, ArrayXXf beta_obs, float rho_obs, ArrayXXf x_obs,
	    ArrayXXf y_obs, ArrayXXf z_obs, ArrayXXf a_obs, ArrayXXf b_obs, ArrayXXf c_obs, ArrayXXf d_obs, ArrayXXf lamda_obs_x,
	    ArrayXXf lamda_obs_y, ArrayXXf lamda_obs_z)
	{
	    ArrayXXf temp_x_obs, b_x_obs, temp_y_obs, b_y_obs, temp_z_obs, b_z_obs;
	    ArrayXXf M1(num * num_obs, 1), M2(num * num_obs, 1), randm1, randm2;


	    // x_________________________________
	    temp_x_obs = (d_obs * cos(alpha_obs) * sin(beta_obs)).colwise() * a_obs.col(0);

	    randm1 = x_obs.transpose();
	    randm2 = temp_x_obs.transpose();
	    int k = 0;
	    for (int i = 0; i < M1.rows(); i++)
	    {
		M1(i) = randm1(k);
		M2(i) = randm2(k);
		k++;
		if (k == x_obs.rows() * x_obs.cols())
		    k = 0;
	    }
	    b_x_obs = M1 + M2;

	    // y_____________________________________
	    temp_y_obs = (d_obs * sin(alpha_obs) * sin(beta_obs)).colwise() * b_obs.col(0);

	    randm1 = y_obs.transpose();
	    randm2 = temp_y_obs.transpose();
	    k = 0;
	    for (int i = 0; i < M1.rows(); i++)
	    {
		M1(i) = randm1(k);
		M2(i) = randm2(k);
		k++;
		if (k == y_obs.rows() * y_obs.cols())
		    k = 0;
	    }
	    b_y_obs = M1 + M2;

	    // z______________________________________
	    temp_z_obs = (d_obs * cos(beta_obs)).colwise() * c_obs.col(0);

	    randm1 = z_obs.transpose();
	    randm2 = temp_z_obs.transpose();
	    k = 0;
	    for (int i = 0; i < M1.rows(); i++)
	    {
		M1(i) = randm1(k);
		M2(i) = randm2(k);
		k++;
		if (k == z_obs.rows() * z_obs.cols())
		    k = 0;
	    }
	    b_z_obs = M1 + M2;

	    // __________________________________________

	    ArrayXXf linterm_augment_x, linterm_augment_y, linterm_augment_z;
	    ArrayXXf sol_x, sol_y, sol_z, primal_sol_x, primal_sol_y, primal_sol_z;


	    linterm_augment_x = -lamda_obs_x - rho_obs * (A_obs.transpose().matrix() * b_x_obs.matrix()).array();
	    linterm_augment_y = -lamda_obs_y - rho_obs * (A_obs.transpose().matrix() * b_y_obs.matrix()).array();
	    linterm_augment_z = -lamda_obs_z - rho_obs * (A_obs.transpose().matrix() * b_z_obs.matrix()).array();

	    sol_x = cost_mat_inv.matrix() * stack(-linterm_augment_x, bx_eq, 'v').matrix();
	    sol_y = cost_mat_inv.matrix() * stack(-linterm_augment_y, by_eq, 'v').matrix();
	    sol_z = cost_mat_inv.matrix() * stack(-linterm_augment_z, bz_eq, 'v').matrix();

	    primal_sol_x = sol_x.topRows(nvar - 0);
	    primal_sol_y = sol_y.topRows(nvar - 0);
	    primal_sol_z = sol_z.topRows(nvar - 0);

	    six_var s;
	    s.a = P.matrix() * primal_sol_x.matrix(); // x
	    s.b = P.matrix() * primal_sol_y.matrix(); // y
	    s.c = P.matrix() * primal_sol_z.matrix(); // z
	    s.d = Pdot.matrix() * primal_sol_x.matrix(); // xdot
	    s.e = Pdot.matrix() * primal_sol_y.matrix(); // ydot
	    s.f = Pdot.matrix() * primal_sol_z.matrix(); // zdot

	    return s;
	}
	nine_var init_guess_compute(ArrayXXf P, ArrayXXf Pdot, ArrayXXf Pddot, int num_obs, float x_init, float y_init, float z_init, float vx_init, float vy_init, float vz_init,
	    float x_d, float y_d, float z_d, ArrayXXf x_obs, ArrayXXf y_obs, ArrayXXf z_obs, ArrayXXf a_obs, ArrayXXf b_obs, ArrayXXf c_obs)
	{
	    six_var s;
	    nine_var s_;

	    int maxiter = 30;
	    int nvar = P.cols();
	    int num = P.rows();

	    float weight_xy = 30.0, rho_obs = 1.0, rho_aug;

	    ArrayXXf cost_smoothness, lincost_smoothness(nvar, 1), cost;
	    ArrayXXf A_eq, bx_eq(3, 1), by_eq(3, 1), bz_eq(3, 1);
	    ArrayXXf lamda_obs_x(nvar, 1), lamda_obs_y(nvar, 1), lamda_obs_z(nvar, 1);
	    ArrayXXf A_obs, alpha_obs(num_obs, num), beta_obs(num_obs, num), d_obs(num_obs, num);
	    ArrayXXf cost_mat_inv, res_x(maxiter, 1), res_y(maxiter, 1), res_z(maxiter, 1);
	    ArrayXXf wc_alpha, ws_alpha, wc_beta, ws_beta, c1_d, c2_d, d_temp;
	    ArrayXXf res_x_obs_vec, res_y_obs_vec, res_z_obs_vec;

	    MatrixXf cost_mat;

	    cost_smoothness = weight_xy * (Pddot.transpose().matrix()) * (Pddot.matrix());
	    lincost_smoothness = 0.0;

	    A_eq = stack(stack(P.row(0), Pdot.row(0), 'v'), P.row(P.rows() - 1), 'v');
	    bx_eq << x_init, vx_init, x_d;
	    by_eq << y_init, vy_init, y_d;
	    bz_eq << z_init, vz_init, z_d;

	    lamda_obs_x = 1.0;
	    lamda_obs_y = 1.0;
	    lamda_obs_z = 1.0;

	    // tile
	    A_obs = stack(P, P, 'v');
	    for (int i = 0; i < num_obs - 2; i++)
	    {
		A_obs = stack(A_obs, P, 'v');
	    }

	    alpha_obs = 0.0;
	    beta_obs = 0.0;
	    d_obs = 1.0;

	    ArrayXXf temp(A_eq.rows(), A_eq.rows());
	    temp = 0.0;

	    cost = cost_smoothness + (rho_obs * (A_obs.transpose().matrix()) * (A_obs.matrix())).array();
	    cost_mat = stack(stack(cost, A_eq.transpose(), 'h'), stack(A_eq, temp, 'h'), 'v');
	    // Ax = I (for inverse)
	    MatrixXf I(cost_mat.rows(), cost_mat.cols());
	    I.setIdentity();
	    cost_mat_inv = (cost_mat).householderQr().solve(I);

	    res_x = 1.0;
	    res_y = 1.0;
	    res_z = 1.0;

	    for (int i = 0; i < maxiter; i++)
	    {
		// x(a) y(b) z(c) xdot(d) ydot(e) zdot(f) 
		s = compute_initial_guess(num_obs, num, nvar, P, Pdot, Pddot, A_eq, bx_eq, by_eq,
		    bz_eq, cost_mat_inv, A_obs, alpha_obs, beta_obs, rho_obs, x_obs,
		    y_obs, z_obs, a_obs, b_obs, c_obs, d_obs, lamda_obs_x,
		    lamda_obs_y, lamda_obs_z);

		wc_alpha = (-x_obs).rowwise() + s.a.transpose().row(0); // 100 x 1 .13 x 100
		ws_alpha = (-y_obs).rowwise() + s.b.transpose().row(0);

		alpha_obs = arctan2(ws_alpha.colwise() * a_obs.col(0), wc_alpha.colwise() * b_obs.col(0));

		wc_beta = (-z_obs).rowwise() + s.c.transpose().row(0);
		ws_beta = (wc_alpha) / (cos(alpha_obs));

		beta_obs = arctan2(ws_beta.colwise() / a_obs.col(0), wc_beta.colwise() / c_obs.col(0));


		rho_aug = rho_obs;

		c1_d = rho_aug * ((pow(sin(beta_obs), 2) * pow(cos(alpha_obs), 2)).colwise() * pow(a_obs.col(0), 2) + (pow(sin(alpha_obs), 2) * pow(sin(beta_obs), 2)).colwise() * pow(b_obs.col(0), 2) + pow(cos(beta_obs), 2).colwise() * pow(c_obs.col(0), 2));
		c2_d = rho_aug * ((wc_alpha * sin(beta_obs) * cos(alpha_obs)).colwise() * a_obs.col(0) + (ws_alpha * sin(alpha_obs) * sin(beta_obs)).colwise() * b_obs.col(0) + (wc_beta * cos(beta_obs)).colwise() * c_obs.col(0));

		d_temp = c2_d / c1_d;
		d_obs = maximum(1, d_temp);


		res_x_obs_vec = wc_alpha - (d_obs * cos(alpha_obs) * sin(beta_obs)).colwise() * a_obs.col(0);
		res_y_obs_vec = ws_alpha - (d_obs * sin(alpha_obs) * sin(beta_obs)).colwise() * b_obs.col(0);
		res_z_obs_vec = wc_beta - (d_obs * cos(beta_obs)).colwise() * c_obs.col(0);

		ArrayXXf M1(num * num_obs, 1), M2(num * num_obs, 1), M3(num * num_obs, 1), randm1, randm2, randm3;

		randm1 = res_x_obs_vec.transpose();
		randm2 = res_y_obs_vec.transpose();
		randm3 = res_z_obs_vec.transpose();
		int k = 0;
		for (int i = 0; i < M1.rows(); i++)
		{
		    M1(i) = randm1(k);
		    M2(i) = randm2(k);
		    M3(i) = randm3(k);
		    k++;
		    if (k == res_x_obs_vec.rows() * res_x_obs_vec.cols())
		        k = 0;
		}

		lamda_obs_x = lamda_obs_x - rho_obs * (A_obs.transpose().matrix() * M1.matrix()).array();
		lamda_obs_y = lamda_obs_y - rho_obs * (A_obs.transpose().matrix() * M2.matrix()).array();
		lamda_obs_z = lamda_obs_z - rho_obs * (A_obs.transpose().matrix() * M3.matrix()).array();

		res_x(i) = (res_x_obs_vec).matrix().lpNorm<2>();
		res_y(i) = (res_y_obs_vec).matrix().lpNorm<2>();
		res_z(i) = (res_z_obs_vec).matrix().lpNorm<2>();
	    }
	    s_.a = s.a;
	    s_.b = s.b;
	    s_.c = s.c;
	    s_.d = s.d;
	    s_.e = s.e;
	    s_.f = s.f;
	    s_.g = lamda_obs_x;
	    s_.h = lamda_obs_y;
	    s_.i = lamda_obs_z;

	    return s_;
	}
	ArrayXXf compute_cpsi_fixed(float psi_init, float psidot_init, float psiddot_init, ArrayXXf P, ArrayXXf Pdot, ArrayXXf Pddot)
	{
	    float Pdot_00 = Pdot(0, 0);
	    float Pdot_01 = Pdot(0, 1);
	    float Pddot_00 = Pddot(0, 0);
	    float Pddot_01 = Pddot(0, 1);
	    float Pddot_02 = Pddot(0, 2);

	    float cpsi_1 = psi_init;
	    float cpsi_2 = (psidot_init - cpsi_1 * Pdot_00) / Pdot_01;
	    float cpsi_3 = (psiddot_init - cpsi_1 * Pddot_00 - cpsi_2 * Pddot_01) / Pddot_02;

	    ArrayXXf cpsi_fixed(1, 3);
	    cpsi_fixed << cpsi_1, cpsi_2, cpsi_3;
	    return cpsi_fixed;
	}

	two_var compute_smoothness_cost_psi(int num, int nvar, float weight_psi, ArrayXXf cpsi_fixed, ArrayXXf P, ArrayXXf Pdot, ArrayXXf Pddot)
	{
	    two_var s;

	    ArrayXXf psiddot_offset;
	    ArrayXXf A_smoothness_acc, b_smoothness_acc;

	    float cpsi_1 = cpsi_fixed(0);
	    float cpsi_2 = cpsi_fixed(1);
	    float cpsi_3 = cpsi_fixed(2);


	    psiddot_offset = (cpsi_1 * Pddot.col(0) + cpsi_2 * Pddot.col(1) + cpsi_3 * Pddot.col(2)).bottomRows(num - 1);


	    A_smoothness_acc = (Pddot.bottomRows(num - 1)).rightCols(nvar - 3);
	    b_smoothness_acc = psiddot_offset;

	    s.a = weight_psi * (A_smoothness_acc.transpose()).matrix() * A_smoothness_acc.matrix();
	    s.b = weight_psi * (A_smoothness_acc.transpose()).matrix() * b_smoothness_acc.matrix();
	    return s;
	}

	seven_var compute_psi(ArrayXXf P, ArrayXXf Pdot, ArrayXXf Pddot, int nvar, int num, ArrayXXf cost_smoothness_psi, ArrayXXf lincost_smoothness_psi, ArrayXXf v, ArrayXXf psi_temp,
	    ArrayXXf cpsi_fixed, ArrayXXf lamda_psi, float rho_psi, ArrayXXf lamda_psidot_ineq, float rho_psidot_ineq, ArrayXXf s_psidot_ineq, float phi_max, float psi_term, float weight_psi_term)
	{
	    float g = 10.0;

	    float cpsi_1 = cpsi_fixed(0);
	    float cpsi_2 = cpsi_fixed(1);
	    float cpsi_3 = cpsi_fixed(2);

	    seven_var s;

	    ArrayXXf psi_offset, psidot_offset, A_psi, b_psi, A_psidot_ineq, B_psidot_ineq, b_psidot_ineq;
	    ArrayXXf obj_psidot_ineq, linterm_augment_psidot_ineq, obj_psi, linterm_augment_psi, linterm_lagrangian_psi;
	    MatrixXf cost_psi, lincost_psi, c_psi, sol;

	    psi_offset = (cpsi_1 * P.col(0) + cpsi_2 * P.col(1) + cpsi_3 * P.col(2)).bottomRows(num - 1);
	    psidot_offset = (cpsi_1 * Pdot.col(0) + cpsi_2 * Pdot.col(1) + cpsi_3 * Pdot.col(2)).bottomRows(num - 1);

	    A_psi = (P.rightCols(nvar - 3)).bottomRows(num - 1);
	    b_psi = psi_temp.bottomRows(num - 1) - psi_offset;

	    A_psidot_ineq = stack((Pdot.bottomRows(num - 1)).rightCols(nvar - 3), -1 * (Pdot.bottomRows(num - 1)).rightCols(nvar - 3), 'v');
	    B_psidot_ineq = stack(((g * tan(phi_max)) / v.bottomRows(num - 1)) - psidot_offset, ((g * tan(phi_max)) / v.bottomRows(num - 1)) + psidot_offset, 'v');

	    b_psidot_ineq = B_psidot_ineq - s_psidot_ineq - (lamda_psidot_ineq / rho_psidot_ineq);

	    obj_psidot_ineq = rho_psidot_ineq * ((A_psidot_ineq.transpose()).matrix() * A_psidot_ineq.matrix());
	    linterm_augment_psidot_ineq = -rho_psidot_ineq * ((A_psidot_ineq.transpose()).matrix() * b_psidot_ineq.matrix());

	    obj_psi = rho_psi * ((A_psi.transpose()).matrix() * A_psi.matrix());
	    linterm_augment_psi = -rho_psi * ((A_psi.transpose()).matrix() * b_psi.matrix());
	    linterm_lagrangian_psi = ((A_psi.transpose()).matrix() * lamda_psi.matrix());

	    cost_psi = -(cost_smoothness_psi + obj_psi + obj_psidot_ineq);
	    lincost_psi = lincost_smoothness_psi + linterm_augment_psi + linterm_lagrangian_psi + linterm_augment_psidot_ineq;


	    sol = (cost_psi).fullPivLu().solve(lincost_psi);

	    c_psi = stack(cpsi_fixed.transpose(), sol, 'v');

	    s.a = P.matrix() * c_psi; // psi
	    s.b = Pdot.matrix() * c_psi; // psidot

	    s.c = (A_psi.matrix() * sol);
	    s.c = s.c - b_psi; // res_psi
	    s.d = lamda_psi + s.c * rho_psi;    // lamda_psi


	    ArrayXXf temp;
	    temp = A_psidot_ineq.matrix() * sol;

	    s.e = maximum(0, -temp + B_psidot_ineq - lamda_psidot_ineq / rho_psidot_ineq); // s_psi_dot_ineq
	    s.f = temp - B_psidot_ineq + s.e; // res_psidot_ineq
	    s.g = lamda_psidot_ineq + rho_psidot_ineq * s.f; // lamda_psidot_ineq


	    return s;
	}

	six_var compute_x(int num_obs, ArrayXXf P, ArrayXXf Pdot, ArrayXXf Pddot, ArrayXXf cost_smoothness, ArrayXXf A_eq, ArrayXXf b_eq_x,
	    ArrayXXf b_eq_y, ArrayXXf b_eq_z, ArrayXXf A_nonhol, ArrayXXf psi, ArrayXXf gamma_, ArrayXXf v, float rho_nonhol, ArrayXXf lamda_nonhol_x,
	    ArrayXXf lamda_nonhol_y, ArrayXXf lamda_nonhol_z, ArrayXXf A_goal, ArrayXXf b_goal_x, ArrayXXf b_goal_y, ArrayXXf b_goal_z, float weight_goal,
	    ArrayXXf d_obs, ArrayXXf alpha_obs, ArrayXXf beta_obs, ArrayXXf a_obs, ArrayXXf b_obs, ArrayXXf c_obs, ArrayXXf x_obs, ArrayXXf y_obs, ArrayXXf z_obs,
	    ArrayXXf A_obs, float rho_obs)
	{
	    int nvar = P.cols();
	    int num = P.rows();

	    ArrayXXf b_nonhol_x, b_nonhol_y, b_nonhol_z;

	    b_nonhol_x = v * cos(psi) * cos(gamma_);
	    b_nonhol_y = v * sin(psi) * cos(gamma_);
	    b_nonhol_z = -v * sin(gamma_);


	    ArrayXXf temp_x_obs, b_x_obs, temp_y_obs, b_y_obs, temp_z_obs, b_z_obs;
	    ArrayXXf M1(num * num_obs, 1), M2(num * num_obs, 1), randm1, randm2;


	    // x_________________________________
	    temp_x_obs = (d_obs * cos(alpha_obs) * sin(beta_obs)).colwise() * a_obs.col(0);

	    randm1 = x_obs.transpose();
	    randm2 = temp_x_obs.transpose();
	    int k = 0;
	    for (int i = 0; i < M1.rows(); i++)
	    {
		M1(i) = randm1(k);
		M2(i) = randm2(k);
		k++;
		if (k == x_obs.rows() * x_obs.cols())
		    k = 0;
	    }
	    b_x_obs = M1 + M2;

	    // y_____________________________________
	    temp_y_obs = (d_obs * sin(alpha_obs) * sin(beta_obs)).colwise() * b_obs.col(0);

	    randm1 = y_obs.transpose();
	    randm2 = temp_y_obs.transpose();
	    k = 0;
	    for (int i = 0; i < M1.rows(); i++)
	    {
		M1(i) = randm1(k);
		M2(i) = randm2(k);
		k++;
		if (k == y_obs.rows() * y_obs.cols())
		    k = 0;
	    }
	    b_y_obs = M1 + M2;

	    // z______________________________________
	    temp_z_obs = (d_obs * cos(beta_obs)).colwise() * c_obs.col(0);

	    randm1 = z_obs.transpose();
	    randm2 = temp_z_obs.transpose();
	    k = 0;
	    for (int i = 0; i < M1.rows(); i++)
	    {
		M1(i) = randm1(k);
		M2(i) = randm2(k);
		k++;
		if (k == z_obs.rows() * z_obs.cols())
		    k = 0;
	    }
	    b_z_obs = M1 + M2;

	    // __________________________________________
	    ArrayXXf obj, cost_mat_inv;
	    MatrixXf cost_mat;

	    ArrayXXf temp(A_eq.rows(), A_eq.rows());
	    temp = 0.0;

	    obj = cost_smoothness + rho_obs * (A_obs.transpose().matrix() * A_obs.matrix()).array() + rho_nonhol * (A_nonhol.transpose().matrix() * A_nonhol.matrix()).array() + weight_goal * (A_goal.transpose().matrix() * A_goal.matrix()).array();
	    cost_mat = stack(stack(obj, A_eq.transpose(), 'h'), stack(A_eq, temp, 'h'), 'v');

	    // Ax = I (for inverse)
	    MatrixXf I(cost_mat.rows(), cost_mat.cols());
	    I.setIdentity();
	    cost_mat_inv = (cost_mat).householderQr().solve(I);

	    ArrayXXf linterm_augment_x, linterm_augment_y, linterm_augment_z;
	    ArrayXXf sol_x, sol_y, sol_z, primal_sol_x, primal_sol_y, primal_sol_z;

	    linterm_augment_x = -lamda_nonhol_x - rho_obs * (A_obs.transpose().matrix() * b_x_obs.matrix()).array() - rho_nonhol * (A_nonhol.transpose().matrix() * b_nonhol_x.matrix()).array() - weight_goal * (A_goal.transpose().matrix() * b_goal_x.matrix()).array();
	    linterm_augment_y = -lamda_nonhol_y - rho_obs * (A_obs.transpose().matrix() * b_y_obs.matrix()).array() - rho_nonhol * (A_nonhol.transpose().matrix() * b_nonhol_y.matrix()).array() - weight_goal * (A_goal.transpose().matrix() * b_goal_y.matrix()).array();
	    linterm_augment_z = -lamda_nonhol_z - rho_obs * (A_obs.transpose().matrix() * b_z_obs.matrix()).array() - rho_nonhol * (A_nonhol.transpose().matrix() * b_nonhol_z.matrix()).array() - weight_goal * (A_goal.transpose().matrix() * b_goal_z.matrix()).array();


	    sol_x = cost_mat_inv.matrix() * (stack(-linterm_augment_x, b_eq_x, 'v')).matrix();
	    sol_y = cost_mat_inv.matrix() * stack(-linterm_augment_y, b_eq_y, 'v').matrix();
	    sol_z = cost_mat_inv.matrix() * stack(-linterm_augment_z, b_eq_z, 'v').matrix();

	    //cout << sol_x;

	    primal_sol_x = sol_x.topRows(nvar - 0);
	    primal_sol_y = sol_y.topRows(nvar - 0);
	    primal_sol_z = sol_z.topRows(nvar - 0);

	    six_var s;
	    s.a = P.matrix() * primal_sol_x.matrix(); // x
	    s.b = P.matrix() * primal_sol_y.matrix(); // y
	    s.c = P.matrix() * primal_sol_z.matrix(); // z
	    s.d = Pdot.matrix() * primal_sol_x.matrix(); // xdot
	    s.e = Pdot.matrix() * primal_sol_y.matrix(); // ydot
	    s.f = Pdot.matrix() * primal_sol_z.matrix(); // zdot

	    return s;
	}
}
