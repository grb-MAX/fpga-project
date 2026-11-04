onerror {quit -force -code 1}
set project_dir [file normalize [pwd]]
set test_dir [file join $project_dir tests]
set local_ini [file join $project_dir modelsim.ini]
if {![file exists $local_ini]} {vmap -c}
vlib [file join $test_dir work_balance_only]
vmap -modelsimini $local_ini work_balance_only [file join $test_dir work_balance_only]
set balance_only_source [file join $project_dir releases 20261028_170500_BalanceOnly_NoAutoswing src pendulum_controller.v]
vlog -modelsimini $local_ini -work work_balance_only -sv $balance_only_source [file join $test_dir tb_balance_only.sv]
vsim -modelsimini $local_ini -voptargs=+acc work_balance_only.tb_balance_only
onfinish stop
run -all
quit -force -code 0
