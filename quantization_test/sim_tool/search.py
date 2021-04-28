from .py_sim import Vehicle, Load
import copy

def search_sto(N, action_mat, gtr, front_car_traj):
    gtr.reset()
    ctrl_cmds = []
    
    for i in range(N):
        dk, _ = gtr.find_closest(gtr.d, gtr.d_list)
        vk, _ = gtr.find_closest(gtr.v, gtr.v_list)
        xk = dk*len(gtr.v_list)+vk
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

def exam_policy(N, gtr, front_car_traj, policy, loose = False, verbose = True):
    gtr.reset()
    cost2go = 0
    valid_ctrl = True

    for k in range(N):
        # _, dc = gtr.find_closest(front_car_traj[k][0],gtr.d_list)
        dc = front_car_traj[k][0]
        # find the corresponding ctrl
        a = gtr.a_list[policy[k]]
        if verbose:
            print('k=%d, dc=%.2f, d=%.2f, v=%.2f, a=%.2f, '%(k, dc, gtr.d, gtr.v, a), end='')
        if k>0:
            penalty, stage1, stage2 = gtr.rl_constraint(k, dc, a)
            if penalty:
                print('%d, red light constraint'%(k))
                # print('dc=%.2f, d=%.2f, v=%.2f, a=%.2f, '%(dc, gtr.d, gtr.v, a))
                print('%.2f, %.2f'%(stage1, stage2))
                valid_ctrl = False

            penalty, stage1 = gtr.dist_constraint(dc)
            if penalty:
                print('%d, distance constraint'%(k))
                # print('dc=%.2f, d=%.2f, v=%.2f, a=%.2f, '%(dc, gtr.d, gtr.v, a))
                print('safety dist: %.2f'%(stage1))
                valid_ctrl = False

        # calculate one running cost
        r_cost = gtr.running_cost(a)
        cost2go += r_cost
        if verbose:
            print('r_cost: %.2f'%(r_cost))
        # walk one step
        gtr.step_forward(a)

    # examine the final state
    # _, dc = gtr.find_closest(front_car_traj[N][0],gtr.d_list)
    dc = front_car_traj[k][0]
    penalty, stage1, stage2 = gtr.rl_constraint(k, dc, a)
    if penalty:
        print('N, red light constraint')
        # print('dc=%.2f, d=%.2f, v=%.2f, a=%.2f, '%(dc, gtr.d, gtr.v, a))
        print('%.2f, %.2f'%(stage1, stage2))
        valid_ctrl = False

    penalty, stage1 = gtr.dist_constraint(dc)
    if penalty:
        print('N, distance constraint')
        # print('dc=%.2f, d=%.2f, v=%.2f, a=%.2f, '%(dc, gtr.d, gtr.v, a))
        print('safety dist: %.2f'%(stage1))
        valid_ctrl = False


    t_cost = gtr.terminal_cost()
    if verbose:
        print('k=10, dc=%.2f, d=%.2f, v=%.2f, '%(dc, gtr.d, gtr.v), end='')
        print("t_cost: %.2f"%(t_cost))
    cost2go += t_cost
    if verbose:
        print('cost to go: %.2f (%.3e)'%(cost2go, cost2go))
        # print('policy is: ', end='')
        # print(policy)
        print('----')

    if valid_ctrl == False and loose == False:
        cost2go = 1e15
        print('====')

    return cost2go