using Random
using JuMP
using CPLEX

Random.seed!(42)

T    = 0:780                 # time slots (integer minutes)
B    = 1:4                   # 1 bus (extendable)
P    = 1:200                  # passengers at CEI
P_r  = 1:100                  # passengers at CRBT2

L      = 1                   # time step
tau    = 25                  # travel time between terminals
c_max  = 20                  # bus capacity
w_max  = 60                  # max waiting time (minutes)

# Big-M
M = maximum(T) + w_max + 1

# Random arrival (Protect infeasible)
latest_board_time = T[end] - tau
safe_arr_upper    = max(first(T), latest_board_time - w_max)
arr   = Dict(p  => rand(first(T):safe_arr_upper) for p  in P)
arr_r = Dict(pr => rand(first(T):safe_arr_upper) for pr in P_r)

println("arr  (CEI) sample: ", first(collect(arr), 5))
println("arr_r(CRBT2) sample: ", first(collect(arr_r), 5))

model = Model(CPLEX.Optimizer)

set_optimizer_attribute(model, "CPX_PARAM_THREADS", 20)
set_optimizer_attribute(model, "CPX_PARAM_MIPEMPHASIS", 1)
set_optimizer_attribute(model, "CPX_PARAM_HEURFREQ", 10)
set_optimizer_attribute(model, "CPX_PARAM_EPGAP", 0.02)
set_optimizer_attribute(model, "CPX_PARAM_CUTSFACTOR", 2)

# Assignment variables
@variable(model, x[i in P,     c in B, t in T], Bin)   # CEI -> CRBT2
@variable(model, x_r[i in P_r, c in B, t in T], Bin)   # CRBT2 -> CEI

# Departure time for each passenger (continuous)
@variable(model, Dep[i in P]       >= 0) #! M3
@variable(model, Dep_r[i in P_r]   >= 0) #! M4

# Bus availability at terminals (binary)
@variable(model, ba[t in T, c in B],   Bin)    # available at CEI at time t   #! M8
@variable(model, ba_r[t in T, c in B], Bin)    # available at CRBT2 at time t #! M8

# Bus departure decision (binary)
@variable(model, bd[t in T, c in B],   Bin)    # depart from CEI at t    #! M6
@variable(model, bd_r[t in T, c in B], Bin)    # depart from CRBT2 at t  #! M7

# Auxiliary waiting-time variables for linear objective
@variable(model, Z_cei[i in P,     c in B, t in T] >= 0)
@variable(model, Z_r[i in P_r,   c in B, t in T]   >= 0)

@objective(model, Min,
    sum(Z_cei[i,c,t] for i in P,   c in B, t in T) +
    sum(Z_r[i,c,t]   for i in P_r, c in B, t in T)
)

# 1) Exact assignment
@constraint(model, [i in P],   sum(x[i,c,t]   for c in B, t in T) == 1) #! F1
@constraint(model, [i in P_r], sum(x_r[i,c,t] for c in B, t in T) == 1) #! F4

# 2) Gate by bd/bd_r and capacity
@constraint(model, [t in T, c in B], sum(x[i,c,t]   for i in P)   <= c_max * bd[t,c])   #! F2
@constraint(model, [t in T, c in B], sum(x_r[i,c,t] for i in P_r) <= c_max * bd_r[t,c]) #! F5 
@constraint(model, [t in T, c in B], sum(x[i,c,t]   for i in P)   <= c_max) #! F3
@constraint(model, [t in T, c in B], sum(x_r[i,c,t] for i in P_r) <= c_max) #! F6
@constraint(model, [i in P,     c in B, t in T], x[i,c,t]   <= bd[t,c])
@constraint(model, [i in P_r,   c in B, t in T], x_r[i,c,t] <= bd_r[t,c])

# 3) Bus must exist at terminal to depart
@constraint(model, [t in T, c in B], bd[t,c]   <= ba[t,c])   #! F7
@constraint(model, [t in T, c in B], bd_r[t,c] <= ba_r[t,c]) #! F8

# 4) Cannot be at both terminals simultaneously
@constraint(model, [t in T, c in B], ba[t,c] + ba_r[t,c] <= 1) 

# 5) Start-of-day: bus available at exactly one terminal
@constraint(model, [c in B], ba[first(T), c] + ba_r[first(T), c] == 1) #! F15

# 6) No boarding in the last τ time slots (ใช้ "ค่าของเวลา" โดยตรง ไม่ index ผ่าน T[..])
T_last_block = (last(T) - tau + L) : last(T)
@constraint(model, [t in T_last_block, c in B], sum(x[i,c,t]   for i in P)   == 0) #! F10
@constraint(model, [t in T_last_block, c in B], sum(x_r[i,c,t] for i in P_r) == 0) #! F13

# 7) Flow equalities of bus state (Control bus per unit movement)
#    ba[t+1] = ba[t] - bd[t] + (t>=tau ? bd_r[t - tau] : 0)
#    ba_r[t+1] = ba_r[t] - bd_r[t] + (t>=tau ? bd[t - tau] : 0)
#    and bus can't depart from both terminals simultaneously
@constraint(model, [t in T, c in B], bd[t,c] + bd_r[t,c] <= 1) #! M3

#! F9 - F12
for c in B
    for t in T[1:end-1]   # every t < last(T)
        if t >= first(T) + tau
            @constraint(model, ba[t+1, c]   == ba[t, c]   - bd[t, c]   + bd_r[t - tau, c]) #! F9
            @constraint(model, ba_r[t+1, c] == ba_r[t, c] - bd_r[t, c] + bd[t - tau, c])   #! F12
        else
            @constraint(model, ba[t+1, c]   == ba[t, c]   - bd[t, c])
            @constraint(model, ba_r[t+1, c] == ba_r[t, c] - bd_r[t, c])
        end
    end
end

# 8) Time-link constraints: if x=1 -> Dep == t  (2 terminals)
@constraint(model, [i in P,   c in B, t in T[1:end-tau]],   Dep[i]   >= t - M*(1 - x[i,c,t])) #! T1
@constraint(model, [i in P,   c in B, t in T[1:end-tau]],   Dep[i]   <= t + M*(1 - x[i,c,t]))
@constraint(model, [i in P_r, c in B, t in T[1:end-tau]],   Dep_r[i] >= t - M*(1 - x_r[i,c,t])) #! T2
@constraint(model, [i in P_r, c in B, t in T[1:end-tau]],   Dep_r[i] <= t + M*(1 - x_r[i,c,t]))

# 9) Waiting-time bound
@constraint(model, [i in P],     Dep[i]   - arr[i]   <= w_max)  #! T3
@constraint(model, [i in P_r],   Dep_r[i] - arr_r[i] <= w_max)  #! T4

# 10) Linearization of Z = (Dep - arr) when x=1 (and 0 when x=0)
for i in P, c in B, t in T
    @constraint(model, Z_cei[i,c,t] >= Dep[i] - arr[i] - M*(1 - x[i,c,t]))
    @constraint(model, Z_cei[i,c,t] <= Dep[i] - arr[i] + M*(1 - x[i,c,t]))
    @constraint(model, Z_cei[i,c,t] <= M * x[i,c,t])
    @constraint(model, Dep[i] >= arr[i] - M*(1 - x[i,c,t]))
end
for i in P_r, c in B, t in T
    @constraint(model, Z_r[i,c,t] >= Dep_r[i] - arr_r[i] - M*(1 - x_r[i,c,t]))
    @constraint(model, Z_r[i,c,t] <= Dep_r[i] - arr_r[i] + M*(1 - x_r[i,c,t]))
    @constraint(model, Z_r[i,c,t] <= M * x_r[i,c,t])
    @constraint(model, Dep_r[i] >= arr_r[i] - M*(1 - x_r[i,c,t]))
end

optimize!(model)
# Note B_d, B_a มาดู Time line การวิ่ง Visualization
println("\nStatus: ", termination_status(model))
if termination_status(model) == MOI.OPTIMAL || termination_status(model) == MOI.FEASIBLE_POINT
    println("Objective value (total waiting): ", objective_value(model))

    println("\nDepartures from CEI (bd[t]=1):")
    for t in T, c in B
        #if value(bd[t,c]) > 0.5
        println("  t=$t, c=$c")
        #end
    end

    println("\nDepartures from CRBT2 (bd_r[t]=1):")
    for t in T, c in B
        # if value(bd_r[t,c]) > 0.5
        println("  t=$t, c=$c")
        # end
    end

    println("\nSample assignments (first 10 at CEI):")
    let shown = 0
        for i in P, c in B, t in T
            if value(x[i,c,t]) > 0.5
                println("  i=$i -> t=$t (CEI), wait=", round(value(Dep[i]) - arr[i], digits=2))
                shown += 1
                if shown >= 10
                    break
                end
            end
        end
    end

    println("\nSample assignments (CRBT2):")
    for i in P_r, c in B, t in T
        if value(x_r[i,c,t]) > 0.5
            println("  i_r=$i -> t=$t (CRBT2), wait=", round(value(Dep_r[i]) - arr_r[i], digits=2))
        end
    end

    println("\nInitial availability:")
    for c in B
        println("  ba[0,$c]   = ", value(ba[first(T),c]))
        println("  ba_r[0,$c] = ", value(ba_r[first(T),c]))
    end
else
    println("Model not optimal. Consider loosening w_max or expanding T.")
end
