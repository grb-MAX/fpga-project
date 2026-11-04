onerror {quit -force -code 1}
set project_dir [file normalize [pwd]]
set test_dir [file join $project_dir tests]
set local_ini [file join $project_dir modelsim.ini]
if {![file exists $local_ini]} {vmap -c}
vlib [file join $test_dir work_sine_trajectory]
vmap -modelsimini $local_ini work_sine_trajectory [file join $test_dir work_sine_trajectory]
set rtl [glob [file join $project_dir src *.v]]
vlog -modelsimini $local_ini -sv -work work_sine_trajectory {*}$rtl [file join $test_dir tb_sine_trajectory.sv]
vsim -modelsimini $local_ini -voptargs=+acc work_sine_trajectory.tb_sine_trajectory
onfinish stop
run -all
quit -force -code 0
