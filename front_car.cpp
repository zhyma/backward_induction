#include "front_car.h"

FCar::FCar(float d2tl_in, float dt_in, float * v_bound_in, float * a_bound_in)
{
    d2tl = d2tl_in;
    dt = dt_in;
    v_bound[0] = v_bound_in[0];
    v_bound[1] = v_bound_in[1];
    a_bound[0] = a_bound_in[0];
    a_bound[1] = a_bound_in[1];
    d = 0;
    v = 0;
    a = 0;
    intention = 1;
    return;
}

float FCar::draw_a_number(float lower, float upper)
{
    // https://stackoverflow.com/questions/43679358/how-to-correctly-use-stdnormal-distribution-in-c11
    float mid = (lower+upper)/2;
    float sigma = (upper-mid)/6;
    std::normal_distribution<float> distri(mid, sigma);
    float x = distri(gen);
    while(true)
    {
        if( (x > lower) && (x < upper))
            break;
    }
    return x;
}

int FCar::make_decision(float percentage)
{
    if ((intention == 0) && d < d2tl)
        intention = 0;
    else if (d > d2tl)
        intention = 1;
    else
    {
        std::uniform_real_distribution<float> distri(.0, 1.0);
        float x = distri(gen);
        if (x < percentage)
            intention = 0;
        else 
            intention = 1;
    }
}

int FCar::sim_step()
{
    float v_b = v_bound[1];
    float d_ = 0;
    float v_ = 0;

    if (a > a_bound[1])
        a = a_bound[1];
    if (a < a_bound[0])
        a = a_bound[0];

    // speed and acceleration is equal or greater than 0
    // or final speed not exceeding upper bound
    if ((v+a*dt >= 0) && (v+a*dt <=v_b))
    {
        d_ = d+v*dt+0.5*a*dt;
        v_ = v+a*dt;
    }
    // speed can't exceed upper bound
    else if (v+a*dt > v_b)
    {
        d_ = d + (v+v_b)/2*((v_b-v)/a) + (dt-(v_b-v)/a)*v_b;
        v_ = v_b;
    }
    // speed can't be negative
    else
    {
        d_ = d-v*v/(2*a);
        v_ = 0;
    }

    d = d_;
    v = v_;
    return 0;
}

int FCar::vehicle_ctrl(bool rl)
{
    float d2stop = d2tl - d;

    // green light or has past the traffic light, accelerate
    if ((rl == false) || (d > d2tl))
    {
        if (v < 10)
            a = draw_a_number(1.0, 2.0);
        if (v >= 10)
            a = draw_a_number( .0, 1.0);

        intention = 1;
    }
    // red light, decellerate or accelerate?
    else
    {
        float a_ref1 = -2.0;
        float a_ref2 = -4.0;
        float a_ref3 = a_bound[0];

        if (v == 0)
            a = 0;
        // have enough distance to decelerate slowly
        else if (d2stop >= fabs(v*v/2/a_ref1))
        {
            // 100% slow down
            intention = 0;
            a = -v*v/d2stop;
            a = draw_a_number(a-1.5, a);
        }
        // distance only enough for decelerate hard
        else if (d2stop >= fabs(v*v/2/a_ref2))
        {
            // 60% slow down
            make_decision(0.8);
            if (intention == 0)
            {
                a = -v*v/d2stop;
                a = draw_a_number(a-1.5, a);
            }
            else
                a = draw_a_number(a_bound[1]-0.5, a_bound[1]);
        }
        // distance may not enough to stop, acceleration then
        else if (d2stop >= fabs(v*v/2/a_ref3))
        {
            // 30% slow down
            make_decision(0.3);
            if (intention == 0)
                a = a_bound[0];
            else
                a = draw_a_number(1.5, 2.0);
        }
        // distance not enough to fully stop, full acceleration then
        else
            a = a_bound[1];
    }

    if (a > a_bound[1])
        a = a_bound[1];
    if (a < a_bound[0])
        a = a_bound[0];
}

int fc_n_step_sim(int iter)
{
    std::ofstream out_file;
    out_file.open("output/front_car_data.csv", std::ios::out);
    out_file << std::setiosflags(std::ios::fixed) << std::setprecision(2);

    float d2tl = 240;
    float dt = 2;
    float v_bound[] = {.0, 18.0};
    float a_bound[] = {-4.0, 2.0};
    // initializing here to make sure random numbers are truely random.
    FCar gtr(d2tl, dt, v_bound, a_bound);

    for (int i = 0; i < iter; ++i)
    {
        float t = 0;
        float d = 54;

        float rl_start = 12;
        float rl_end = rl_start + 30;

        gtr.d = 0;
        gtr.v = 0;
        bool rl;

        for (int k = 0; k < 24; ++k)
        {
            if ((t >= rl_start) && (t <= rl_end))
                rl = true;
            else
                rl = false;

            gtr.vehicle_ctrl(rl);
            gtr.sim_step();
            t += dt;
            if (k > 0)
            {
                //write to file
                out_file << gtr.d << "," << gtr.v << ",";
                out_file << gtr.a << "," << gtr.intention << std::endl;
            }
        }
        out_file << "end" << std::endl;

        if ((i%(iter/10))==0)
            std::cout << i/(iter/10) << "0%...";

    }
    out_file.close();
    return 0;
}