from .py_sim import Vehicle, Load
import copy

def search_sto(N, action_mat, gtr, front_car_traj):
    gtr.reset()
    ctrl_cmds = []
    
    for i in range(N):
        dk, _ = gtr.find_closest(gtr.d, gtr.d_list)
        vk, _ = gtr.find_closest(gtr.v, gtr.v_list)
        xk = dk*32+vk
        # read one front car state
        dck, _ = gtr.find_closest(front_car_traj[i][0],gtr.d_list)
        intention = front_car_traj[i][1]
        wk = dck*2 + intention
        # find the corresponding ctrl
        ctrl_cmds.append(int(action_mat[i,xk,wk]))
        a = gtr.a_list[ctrl_cmds[-1]]
        # walk one step
        gtr.step_forward(a)

    return ctrl_cmds

def iterate_action(N, gtr, front_car_traj):
    gtr.reset()
    min_cost =1e20
    best_policy = []

    for i in range(32**N):
        cost2go = 0
        policy_list = []
        gtr.reset()
        valid_ctrl = True

        for k in range(N):
            _, dc = gtr.find_closest(front_car_traj[k][0],gtr.d_list)
            ak = int(i/(32**(k)))%32
            a = gtr.a_list[ak]

            if k>0:
                if gtr.rl_constraint(k, dc, a):
                    valid_ctrl = False
                    break
                if gtr.dist_constraint(dc):
                    valid_ctrl = False
                    break

            policy_list.append(ak)
            r_cost = gtr.running_cost(a)

            cost2go += r_cost
            gtr.step_forward(a)
        
        _, dc = gtr.find_closest(front_car_traj[N][0],gtr.d_list)
        if gtr.rl_constraint(N, dc, gtr.a_min):
            valid_ctrl = False
            continue
        if gtr.dist_constraint(dc):
            valid_ctrl = False
            continue

        if valid_ctrl == True:
            t_cost = gtr.terminal_cost()
            cost2go += t_cost
            if cost2go < min_cost:
                min_cost = cost2go
                best_policy = copy.deepcopy(policy_list)

    return best_policy

def exam_policy(N, gtr, front_car_traj, policy):
    gtr.reset()
    cost2go = 0
    valid_ctrl = True

    
    for k in range(N):
        _, dc = gtr.find_closest(front_car_traj[k][0],gtr.d_list)
        # find the corresponding ctrl
        a = gtr.a_list[policy[k]]
        print('dc=%.2f, d=%.2f, v=%.2f, a=%.2f, '%(dc, gtr.d, gtr.v, a), end='')
        if k>0:
            if gtr.rl_constraint(k, dc, a):
                print('Optimal control is not valid, hits the red light constraint')
                valid_ctrl = False
                break
            if gtr.dist_constraint(dc):
                print('Optimal control is not valid, hits the distance constraint')
                print('safety dist: dc-v*t_tcc-3-d = %.2f-%.2f*3-3-%.2f = %.2f'\
                    %(dc, gtr.v, gtr.d, dc-gtr.v*3-3-gtr.d))
                valid_ctrl = False
                break
        # calculate one running cost
        r_cost = gtr.running_cost(a)
        cost2go += r_cost
        print('r_cost: %.2f'%(r_cost))
        # walk one step
        gtr.step_forward(a)

    # examine the final state
    _, dc = gtr.find_closest(front_car_traj[N][0],gtr.d_list)
    if gtr.rl_constraint(N, dc, gtr.a_min):
        print('Optimal control is not valid, hits the red light constraint')
        valid_ctrl = False
    if gtr.dist_constraint(dc):
        print('Optimal control is not valid, hits the distance constraint')
        print('safety dist: dc-v*t_tcc-3-d = %.2f-%.2f*3-3-%.2f = %.2f'\
            %(dc, gtr.v, gtr.d, dc-gtr.v*3-3-gtr.d))
        valid_ctrl = False

    if valid_ctrl == True:
        # print('')
        # terminal cost
        t_cost = gtr.terminal_cost()
        print('dc=%.2f, d=%.2f, v=%.2f, '%(dc, gtr.d, gtr.v), end='')
        print("t_cost: %.2f"%(t_cost))
        cost2go += t_cost
        print('cost to go: %.2f (%.3e)'%(cost2go, cost2go))
        print('policy is: ', end='')
        print(policy)
        print('\n')