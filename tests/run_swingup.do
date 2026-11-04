onerror {quit -force -code 1}
set project_dir [file normalize [pwd]]
set test_dir [file join $project_dir tests]
set local_ini [file join $project_dir modelsim.ini]
if {![file exists $local_ini]} {vmap -c}
if {[file exists [file join $test_dir work_swingup]]} {vdel -lib [file join $test_dir work_swingup] -all}
vlib [file join $test_dir work_swingup]
vmap -modelsimini $local_ini work_swingup [file join $test_dir work_swingup]
set swingup_source [file join $project_dir releases 20261028_140000_Before_BalanceOnly_Restore src pendulum_controller.v]
vlog -modelsimini $local_ini -work work_swingup -sv $swingup_source [file join $test_dir tb_swingup.sv]
vsim -modelsimini $local_ini -voptargs=+acc work_swingup.tb_swingup
onfinish stop
run -all
quit -force -code 0
