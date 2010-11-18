# file: tanh_model.py
#
# ========================
#
# tanh test bed
#
# * run a simple model to turn tanh on high
# * emulate bigm_model
#
# ========================

from realpaver_api2 import *


## Part 1: set "indic = (a > 5)"

# add_constant("a", 10)
# add_variable("indic", 0, 1)
## indic = (a > 5)
# quick_constraint("(tanh(a - 5.0)+1) * 0.5 = indic")

## Part 2: test many tanh statements

# velocity profile
times = range(9)

# 5: ~0s
# 6: 1.5s
# 7: 4.7s
# 8: 29s
# 9: 6s

N = len(times)

C = 3.5
x = [C]*N
M = C*N

add_constant("x", x, times)

add_set("times", times)
after_first = times[1:]
add_set("after_first", after_first)

add_variable("acc", 0.0, M, times)
add_variable("charging", 0, 1, times)
add_variable("max_acc", 11, 20)
add_variable("min_acc", 8, 10)

quick_constraint("acc[0] = 0")
quick_constraint("acc[t] = acc[t-1] + charging[t] * x[t]  - (1 - charging[t]) * x[t] for t in after_first")

# Oh! Gotta be careful with indices

## below_max[t] = (max_acc > acc[t-1])
add_variable("below_max", 0, 1, times)
quick_constraint("below_max[0] = 1")
quick_constraint("(tanh(max_acc - acc[t-1])+1) * 0.5 = below_max[t] for t in after_first")

## above_min[t] =  (acc[t-1] > min_acc)
add_variable("above_min", 0, 1, times)
add_variable("below_min", 0, 1, times)
quick_constraint("above_min[0] = 0")
quick_constraint("below_min[0] = 1")
quick_constraint("(tanh(acc[t-1] - min_acc)+1) * 0.5 = above_min[t] for t in after_first")
quick_constraint("below_min[t] = 1 - above_min[t] for t in times")

quick_constraint("charging[0] = 1")
quick_constraint("charging[t] = charging[t-1] * below_max[t] * (1 - below_min[t]) + below_min[t] for t in after_first")

# obj function
add_variable("obj", 0, "+oo")
quick_constraint("obj = sum(charging, times)")
quick_constraint("obj < %s" % (2*N))


# Dump output to text file
rendered = render()
foo = open("foo2.txt", "w")
foo.write(rendered)
foo.close()

# print rendered
print "Finished rendering to foo2.txt"