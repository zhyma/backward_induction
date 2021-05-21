from subprocess import Popen, PIPE
import sys

with open('log.txt', 'w') as f:
    # for trials in [1000, 10000, 20000, 50000]:
    for trials in [50000]:
        for d in [5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60]:

            title = "number of trials: " + str(trials) + ", d=" + str(d)
            print(title)
            f.write(title)

            gen_cmd = ["python3","front_car_sim_fix_w.py", str(trials), str(d)]
            gen = Popen(gen_cmd, stdout=PIPE, stderr=PIPE)
            text = gen.communicate()[0].decode('utf-8')
            print(text)
            f.write(text)

            solve_cmd = ["./build/dp_solver", "one_step", "gpu"]
            solve = Popen(solve_cmd, stdout=PIPE, stderr=PIPE)
            text = solve.communicate()[0].decode('utf-8')
            print(text)
            f.write(text)

            # check_cmd = ["python3", "value_vs_cost.py", str(trials)]
            check_cmd = ["python3", "value_vs_cost_vs_rule.py", str(trials)]
            check = Popen(check_cmd, stdout=PIPE, stderr=PIPE)
            text = check.communicate()[0].decode('utf-8')
            print(text)
            f.write(text)

            print("====")
            f.write("====")

