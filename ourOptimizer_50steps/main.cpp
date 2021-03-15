#include "yaml-cpp/yaml.h"
#include <iostream>
#include <time.h>
#include <fstream>
#include <cstdlib>
#include <iomanip> 
#include <cmath>
#include <string>
#include "eigen-3.3.8/Eigen/Dense"
#include "FWV_optim.h"
#define pi M_PI

using namespace Eigen;
using namespace std;
using namespace optim;

ofstream outdata;

three_var PPP;									// P, Pdot, Pddot
two_var smoothness_psi;
nine_var sol_guess;
six_var sol_xyz;
seven_var sol_psi;

int num = 50, maxiter = 300, num_obs = 13;
int nvar, collisionFlag = 0;
float t_fin = 11.0, t = t_fin / num;
float weight_xy = 30.0, weight_goal = 4000.0, rho_nonhol = 100.0, rho_obs = 1.0, rho_aug, rho_psi = 1.0;
float weight_psi = 100.0, weight_gamma = 100.0, weight_phi = 1.0, weight_v = 1.0, weight_psi_term = 0.01;

float rho_psidot_ineq = 1.0;

ArrayXXf tot_time(num, 1);
ArrayXXf x_obs(num_obs, num), y_obs(num_obs, num), z_obs(num_obs, num);
ArrayXXf a_obs(num_obs, 1), b_obs(num_obs, 1), c_obs(num_obs, 1);
ArrayXXf obsx(num_obs, 1), obsy(num_obs, 1), obsz(num_obs, 1);


ArrayXXf psi_guess, gamma_guess, v_guess;
ArrayXXf cost_smoothness, A_goal, b_goal_x(1, 1), b_goal_y(1, 1), b_goal_z(1, 1);
ArrayXXf A_eq, b_eq_x(3, 1), b_eq_y(3, 1), b_eq_z(3, 1), A_nonhol;
ArrayXXf lamda_nonhol_x(11, 1), lamda_nonhol_y(11, 1), lamda_nonhol_z(11, 1);
ArrayXXf psi(num, 1), gamma_(num, 1), v(num, 1), res_x(maxiter, 1), res_y(maxiter, 1), res_z(maxiter, 1);
ArrayXXf wc_alpha, ws_alpha, wc_beta, ws_beta, c1_d, c2_d, d_temp;
ArrayXXf alpha_obs(num_obs, num), beta_obs(num_obs, num), d_obs(num_obs, num);
ArrayXXf res_x_obs(maxiter, 1), res_y_obs(maxiter, 1), res_z_obs(maxiter, 1);
ArrayXXf A_aug, A_obs, cpsi_fixed, d_min(maxiter, 1);
ArrayXXf psidot(num, 1), psi_temp;
ArrayXXf lamda_psi(num - 1, 1), lamda_psidot_ineq(2 * (num - 1), 1), s_psidot_ineq(2 * (num - 1), 1);
ArrayXXf res_psidot_ineq(maxiter, 1), res_psi(maxiter, 1);
ArrayXXf b_nonhol_x, b_nonhol_y, b_nonhol_z;
ArrayXXf res_nonhol_x, res_nonhol_y, res_nonhol_z;
ArrayXXf res_x_obs_vec, res_y_obs_vec, res_z_obs_vec;
ArrayXXf res_vec_x, res_vec_y, res_vec_z;
ArrayXXf res_goal_x(1, 1), res_goal_y(1, 1), res_goal_z(1, 1);

float g = 10.0, phi_max = 40.0 * pi / 180.0, gamma_max = 40.0 * pi / 180.0;
float a_max = 3.0, v_min = 12.0, v_max = 16.0;
float v_ref = (v_min + v_max) / 2.0;

float x_init = -50.0,
y_init = 0.0,
z_init = 20.0;

float x_d = 150.0,
y_d = 100.0,
z_d = 25.0;

float v_init = 12.0,
vdot_init = 0.0;

float phi_init = 0.0;

float gamma_init = 0.0,
gammadot_init = 0.0,
gammaddot_init = 0.0;

float psi_init = 0.0,
psidot_init = (g / v_init) * tan(phi_init),
psiddot_init = 0.0,
psi_term;

float vx_init = v_init * cos(psi_init) * cos(gamma_init),
vy_init = v_init * sin(psi_init) * cos(gamma_init),
vz_init = -v_init * sin(gamma_init);

float vxdot_init = 0.0, vydot_init = 0.0, vzdot_init = 0.0;
float delta = 1.05;



void initObs(ArrayXXf vx, ArrayXXf vy, ArrayXXf vz, float dt);
void compute_bernstein(float t_final);
void meh(int c);
void update_psi(int i);
void check_intersection(ArrayXXf X, ArrayXXf Y, ArrayXXf Z);

int main()
{ 
	YAML::Node map = YAML::LoadFile("config.yaml");
	//num = map["configuration"]["steps"].as<int>();
	x_init = map["configuration"]["x_init"].as<float>();
 	y_init = map["configuration"]["y_init"].as<float>();
	z_init = map["configuration"]["z_init"].as<float>();
	psi_init = map["configuration"]["psi_init"].as<float>() * pi;
	
	x_d = map["configuration"]["x_goal"].as<float>();
	y_d = map["configuration"]["y_goal"].as<float>();
	z_d = map["configuration"]["z_goal"].as<float>();

    vx_init = v_init * cos(psi_init) * cos(gamma_init);
    vy_init = v_init * sin(psi_init) * cos(gamma_init);
    vz_init = -v_init * sin(gamma_init);

    ArrayXXf vx_obs(num_obs, 1);
    ArrayXXf vy_obs(num_obs, 1);
    ArrayXXf vz_obs(num_obs, 1);

    vx_obs = 0.0;// << -0.1, -0.5, -1.0, -0.4, -1.2, -0.3, -0.9, -1.0, -1.0, -1.5, -1.5, -1.0, -1.5;
    vy_obs = 0.0;
    vz_obs = 0.0;

    clock_t start, end;

    t = t_fin / num;
    start = clock();
    compute_bernstein(t_fin);
    initObs(vx_obs, vy_obs, vz_obs, t);


    // x(a) y(b) z(c) xdot(d) ydot(e) zdot(f) lamda_x(g) lamda_y(h) lamda_z(i)
    sol_guess = init_guess_compute(PPP.a, PPP.b, PPP.c, num_obs, x_init, y_init, z_init, vx_init, vy_init, vz_init,
        x_d, y_d, z_d, x_obs, y_obs, z_obs, a_obs, b_obs, c_obs);
    psi_guess = arctan2(sol_guess.e, sol_guess.d);


    gamma_guess = arctan2(-sol_guess.f, sol_guess.d * cos(psi_guess) + sol_guess.e * sin(psi_guess));
    v_guess = sqrt(pow(sol_guess.d, 2) + pow(sol_guess.e, 2) + pow(sol_guess.f, 2));

    v_guess = clip(v_min, v_max, v_guess);


    float arc_length = sqrt(pow(diff(sol_guess.a), 2) + pow(diff(sol_guess.b), 2) + pow(diff(sol_guess.c), 2)).sum();
    t_fin = arc_length / v_min;
    t = t_fin / num;

    cout << "dt in secs= " << t_fin/num << endl;
	cout << "Planning time in secs = " << t_fin << endl;
    compute_bernstein(t_fin);
    initObs(vx_obs, vy_obs, vz_obs, t);

    cost_smoothness = weight_xy * PPP.c.transpose().matrix() * PPP.c.matrix();
    A_goal = PPP.a.row(PPP.a.rows() - 1);

    b_goal_x = x_d;
    b_goal_y = y_d;
    b_goal_z = z_d;

    A_eq = stack(PPP.a.row(0), PPP.b.row(0), 'v');
    A_eq = stack(A_eq, PPP.c.row(0), 'v');

    b_eq_x << x_init, vx_init, vxdot_init;
    b_eq_y << y_init, vy_init, vydot_init;
    b_eq_z << z_init, vz_init, vzdot_init;

    A_nonhol = PPP.b;
    lamda_nonhol_x = 0.0;
    lamda_nonhol_y = 0.0;
    lamda_nonhol_z = 0.0;

    psi = psi_guess;
    gamma_ = gamma_guess;
    v = v_guess;

    res_x = 1.0;
    res_y = 1.0;
    res_z = 1.0;

    meh(0);

    A_obs = stack(PPP.a, PPP.a, 'v');
    for (int i = 0; i < num_obs - 2; i++)
    {
        A_obs = stack(A_obs, PPP.a, 'v');
    }
    A_aug = stack(A_nonhol, A_obs, 'v');


    res_x_obs = 1.0;
    res_y_obs = 1.0;
    res_z_obs = 1.0;

    psi_term = psi_init;

    cpsi_fixed = compute_cpsi_fixed(psi_init, psidot_init, psiddot_init, PPP.a, PPP.b, PPP.c);
    smoothness_psi = compute_smoothness_cost_psi(num, nvar, weight_psi, cpsi_fixed, PPP.a, PPP.b, PPP.c);
    d_min = 1.0;

    psidot = psidot_init;
    lamda_psi = 1.0;
    s_psidot_ineq = 1.0;
    lamda_psidot_ineq = 1.0;
    res_psidot_ineq = 1.0;
    res_psi = 1.0;

    ArrayXXf dataX(maxiter, num), dataY(maxiter, num), dataZ(maxiter, num);
    start = clock();
    for (int i = 0; i < maxiter; i++)
    {

        // x(a) y(b) z(c) xdot(d) ydot(e) zdot(f)
        sol_xyz = compute_x(num_obs, PPP.a, PPP.b, PPP.c, cost_smoothness, A_eq, b_eq_x, b_eq_y, b_eq_z,
            A_nonhol, psi, gamma_, v, rho_nonhol, lamda_nonhol_x, lamda_nonhol_y, lamda_nonhol_z,
            A_goal, b_goal_x, b_goal_y, b_goal_z, weight_goal, d_obs, alpha_obs, beta_obs, a_obs,
            b_obs, c_obs, x_obs, y_obs, z_obs, A_obs, rho_obs);

        //cout << sol_xyz.a;
        //break;
        psi_temp = arctan2(sol_xyz.e, sol_xyz.d);

        // psi(a) psidot(b) res_psi(c) lamda_psi(d) s_psidot_ineq(e) res_psidot_ineq(f) lamda_psidot_ineq(g)
        sol_psi = compute_psi(PPP.a, PPP.b, PPP.c, nvar, num, smoothness_psi.a, smoothness_psi.b, v, psi_temp,
            cpsi_fixed, lamda_psi, rho_psi, lamda_psidot_ineq, rho_psidot_ineq, s_psidot_ineq,
            phi_max, psi_term, weight_psi_term);
        update_psi(i);

        gamma_ = arctan2(-sol_xyz.f, sol_xyz.d * cos(psi) + sol_xyz.e * sin(psi));
        gamma_ = clip(-gamma_max, gamma_max, gamma_);

        v = sqrt(pow(sol_xyz.d, 2) + pow(sol_xyz.e, 2) + pow(sol_xyz.f, 2));
        psidot(0) = psiddot_init;
        psidot.bottomRows(num - 1) = abs(diff(psi)) / t;

        ArrayXXf temp_v, temp_vmin(num, 1);
        temp_vmin = v_min;
        temp_v = minimum(v_max, (g * tan(phi_max)) / (abs(psidot) + 0.0001));
        v = clip2(temp_vmin, temp_v, v);

        meh(1);
        d_min(i) = d_temp.minCoeff();

        b_nonhol_x = v * cos(psi) * cos(gamma_);
        b_nonhol_y = v * sin(psi) * cos(gamma_);
        b_nonhol_z = -v * sin(gamma_);

        res_nonhol_x = sol_xyz.d - b_nonhol_x;
        res_nonhol_y = sol_xyz.e - b_nonhol_y;
        res_nonhol_z = sol_xyz.f - b_nonhol_z;

        res_x_obs_vec = wc_alpha - (d_obs * cos(alpha_obs) * sin(beta_obs)).colwise() * a_obs.col(0);
        res_y_obs_vec = ws_alpha - (d_obs * sin(alpha_obs) * sin(beta_obs)).colwise() * b_obs.col(0);
        res_z_obs_vec = wc_beta - (d_obs * cos(beta_obs)).colwise() * c_obs.col(0);


        res_goal_x(0) = sol_xyz.a(sol_xyz.a.rows() * sol_xyz.a.cols() - 1) - x_d;
        res_goal_y(0) = sol_xyz.b(sol_xyz.b.rows() * sol_xyz.b.cols() - 1) - y_d;
        res_goal_z(0) = sol_xyz.c(sol_xyz.c.rows() * sol_xyz.c.cols() - 1) - z_d;

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

        lamda_nonhol_x = lamda_nonhol_x - rho_obs * (A_obs.transpose().matrix() * M1.matrix()).array() - rho_nonhol * (A_nonhol.transpose().matrix() * res_nonhol_x.matrix()).array();
        lamda_nonhol_y = lamda_nonhol_y - rho_obs * (A_obs.transpose().matrix() * M2.matrix()).array() - rho_nonhol * (A_nonhol.transpose().matrix() * res_nonhol_y.matrix()).array();
        lamda_nonhol_z = lamda_nonhol_z - rho_obs * (A_obs.transpose().matrix() * M3.matrix()).array() - rho_nonhol * (A_nonhol.transpose().matrix() * res_nonhol_z.matrix()).array();


        res_x_obs(i) = (res_x_obs_vec).matrix().lpNorm<2>();
        res_y_obs(i) = (res_y_obs_vec).matrix().lpNorm<2>();
        res_z_obs(i) = (res_z_obs_vec).matrix().lpNorm<2>();

        res_x(i) = (res_nonhol_x).matrix().lpNorm<2>();
        res_y(i) = (res_nonhol_y).matrix().lpNorm<2>();
        res_z(i) = (res_nonhol_z).matrix().lpNorm<2>();


        rho_nonhol = rho_nonhol * delta;

        if (d_temp.minCoeff() < 0.99)
            rho_obs = rho_obs * delta;

        if (res_psidot_ineq(i) > 0.004)
            rho_psidot_ineq = rho_psidot_ineq * delta;

        if (res_psi(i) > 0.004)
            rho_psi = rho_psi * delta;

        dataX.row(i) = sol_xyz.a.transpose();
        dataY.row(i) = sol_xyz.b.transpose();
        dataZ.row(i) = sol_xyz.c.transpose();
    }
    end = clock();

    outdata.open("proposed_dataX_each_iter.dat");
    outdata << dataX;
    outdata.close();
    outdata.open("proposed_dataY_each_iter.dat");
    outdata << dataY;
    outdata.close();
    outdata.open("proposed_dataZ_each_iter.dat");
    outdata << dataZ;
    outdata.close();

    outdata.open("obs_dataX_each_iter.dat");
    outdata << x_obs;
    outdata.close();
    outdata.open("obs_dataY_each_iter.dat");
    outdata << y_obs;
    outdata.close();
    outdata.open("obs_dataZ_each_iter.dat");
    outdata << z_obs;
    outdata.close();

    float time_taken = double(end - start) / double(CLOCKS_PER_SEC);
    cout << "TIME TAKEN in secs = " << setprecision(5) << time_taken << endl;

    outdata.open("data.dat");
    for (int i = 0; i < num; i++)
    {
        // cout << sol_xyz.a(i) << " " << sol_xyz.b(i) << " " << sol_xyz.c(i) << endl;
        outdata << sol_xyz.a(i) << " " << sol_xyz.b(i) << " " << sol_xyz.c(i) << endl;
    }
    outdata.close();



    check_intersection(sol_xyz.a, sol_xyz.b, sol_xyz.c);
	if(collisionFlag)
		cout << "Collision!! \n \n";
	else
		cout << "No Collision!! \n \n";

    outdata.open("data2.dat");
    outdata << x_init << " " << y_init << " " << z_init << endl;
    outdata << x_d << " " << y_d << " " << z_d << endl;
    outdata.close();

    ArrayXXf acc(num, 1);
    acc(0) = (v(0) - v_init) / t;
    acc.bottomRows(num - 1) = diff(v) / t;

    cout << "Norm of linear acceleration = " << (acc.matrix()).lpNorm<2>() << endl;

    ArrayXXf phi, phidot(num, 1);
    phi = atan(psidot * v / g);
    phidot(0) = (phi(0) - phi_init) / t;
    phidot.bottomRows(num - 1) = diff(phi) / t;

    cout << "Norm of angular (phidot) acceleration = " << (phidot.matrix()).lpNorm<2>() << endl;

    ArrayXXf gammadot(num, 1);
    gammadot(0) = (gamma_(0) - gamma_init) / t;
    gammadot.bottomRows(num - 1) = diff(gamma_) / t;

    cout << "Norm of angular (gammadot) acceleration = " << (gammadot.matrix()).lpNorm<2>() << endl;

    double final_cost;
    final_cost = sqrt(pow(sol_xyz.a(num - 1) - x_d, 2) + pow(sol_xyz.b(num - 1) - y_d, 2) + pow(sol_xyz.c(num - 1) - z_d, 2));

    cout << "Final position cost = " << final_cost << endl;
    outdata.open("config.dat");
    outdata << "x_init, y_init, z_init, psi_init = " << x_init << " " << y_init << " " << z_init << " " << psi_init << endl << endl;
    outdata << "x_g, y_g, z_g, steps= " << x_d << " " << y_d << " " << z_d << " " << num << endl << endl;
    outdata << "Norm of linear acceleration = " << (acc.matrix()).lpNorm<2>() << endl << endl;
    outdata << "Norm of angular (phidot) acceleration = " << (phidot.matrix()).lpNorm<2>() << endl << endl;
    outdata << "TIME TAKEN in secs = " << setprecision(5) << time_taken << endl << endl;
    outdata << "Final Position Cost = " << final_cost << endl << endl;
    outdata << "Planning time in secs = " << t_fin << endl << endl;
    outdata.close();

    outdata.open("xyz_ypr_last_iter.dat");
    outdata << sol_xyz.a.transpose() << endl;
    outdata << sol_xyz.b.transpose() << endl;
    outdata << sol_xyz.c.transpose() << endl;
    outdata << psi.transpose() << endl;
    outdata << gamma_.transpose() << endl;
    outdata << phi.transpose() << endl;
    outdata.close();

    /*outdata.open("config_10_xyz.dat");
    for (int i = 0; i <  num; i++)
    {
       // cout << sol_xyz.a(i) << " " << sol_xyz.b(i) << " " << sol_xyz.c(i) << endl;
        outdata << sol_xyz.a(i) << " " << sol_xyz.b(i) << " " << sol_xyz.c(i) << endl;
    }
    outdata.close();*/
    outdata.open("vdot.dat");
    for (int i = 0; i < num; i++)
        outdata << i << " " << acc(i) << endl;
    outdata.close();
    outdata.open("phidot.dat");
    for (int i = 0; i < num; i++)
        outdata << i << " " << phidot(i) << endl;
    outdata.close();
    outdata.open("gammadot.dat");
    for (int i = 0; i < num; i++)
        outdata << i << " " << gammadot(i) << endl;
    outdata.close();


    return 1;
}
void update_psi(int i)
{
    psi = sol_psi.a;
    psidot = sol_psi.b;
    lamda_psi = sol_psi.d;
    s_psidot_ineq = sol_psi.e;
    lamda_psidot_ineq = sol_psi.g;
    res_psi(i) = (sol_psi.c.matrix()).lpNorm<2>();
    res_psidot_ineq(i) = (sol_psi.f.matrix()).lpNorm<2>();
}


void compute_bernstein(float t_final)
{
    tot_time = linspace(0.0, t_final, num);
    PPP = bernstein_coeff_order10_new(10.0, tot_time(0), tot_time(num - 1), tot_time, num);
    nvar = PPP.a.cols();
}
void initObs(ArrayXXf vx, ArrayXXf vy, ArrayXXf vz, float dt)
{
    a_obs << 5, 5, 10, 5, 5, 12, 5, 7, 5, 5, 9, 5, 5;
    b_obs << 5, 5, 10, 5, 5, 12, 5, 7, 6, 7, 9, 5, 5;
    c_obs << 35, 35, 65, 85, 25, 50, 55, 35, 50, 65, 75, 40, 45;
    obsx << 10, 45, 50, 70, 110, 15, 40, 60, 100, 120, 80, 100, 0;
    obsy << 55, 50, 10, 70, 45, 100, 110, 105, 95, 80, 120, 0, 20;
    obsz << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

    for (int i = 0; i < num_obs; i++)
    {
        x_obs(i, 0) = obsx(i);
        y_obs(i, 0) = obsy(i);
        z_obs(i, 0) = obsz(i);
    }
    for (int i = 0; i < num_obs; i++)
    {
        for (int j = 1; j < num; j++)
        {
            x_obs(i, j) = x_obs(i, j - 1) + vx(i) * dt;
            y_obs(i, j) = y_obs(i, j - 1) + vy(i) * dt;
            z_obs(i, j) = z_obs(i, j - 1) + vz(i) * dt;
        }
    }
}
void meh(int c)
{
    ArrayXXf X, Y, Z;
    if (c == 1)
    {
        X = sol_xyz.a;
        Y = sol_xyz.b;
        Z = sol_xyz.c;
    }
    else
    {
        X = sol_guess.a;
        Y = sol_guess.b;
        Z = sol_guess.c;
    }
    wc_alpha = (-x_obs).rowwise() + X.transpose().row(0); // 100 x 1 .13 x 100
    ws_alpha = (-y_obs).rowwise() + Y.transpose().row(0);

    alpha_obs = arctan2(ws_alpha.colwise() * a_obs.col(0), wc_alpha.colwise() * b_obs.col(0));

    wc_beta = (-z_obs).rowwise() + Z.transpose().row(0);
    ws_beta = (wc_alpha) / (cos(alpha_obs));

    beta_obs = arctan2(ws_beta.colwise() / a_obs.col(0), wc_beta.colwise() / c_obs.col(0));


    rho_aug = rho_obs;

    c1_d = rho_aug * ((pow(sin(beta_obs), 2) * pow(cos(alpha_obs), 2)).colwise() * pow(a_obs.col(0), 2) + (pow(sin(alpha_obs), 2) * pow(sin(beta_obs), 2)).colwise() * pow(b_obs.col(0), 2) + pow(cos(beta_obs), 2).colwise() * pow(c_obs.col(0), 2));
    c2_d = rho_aug * ((wc_alpha * sin(beta_obs) * cos(alpha_obs)).colwise() * a_obs.col(0) + (ws_alpha * sin(alpha_obs) * sin(beta_obs)).colwise() * b_obs.col(0) + (wc_beta * cos(beta_obs)).colwise() * c_obs.col(0));

    d_temp = c2_d / c1_d;
    d_obs = maximum(1, d_temp);
}

void check_intersection(ArrayXXf X, ArrayXXf Y, ArrayXXf Z)
{
    int inter[num_obs][num];
    for (int j = 0; j < 13; j++)
    {
        for (int i = 0; i < num; i++)
        {
            double temp = pow(X(i) - x_obs(j, i), 2) / pow(a_obs(j), 2) + pow(Y(i) - y_obs(j, i), 2) / pow(b_obs(j), 2) + pow(Z(i) - z_obs(j, i), 2) / pow(c_obs(j), 2);
            inter[j][i] = 0.0;
            if (temp < 1.0)
            {
                inter[j][i] = 1;
                cout << "Obstacle = " << j << " intersection = " << inter[j][i] << endl;
				collisionFlag = 1;
            }
            
        }
       
    }
}

