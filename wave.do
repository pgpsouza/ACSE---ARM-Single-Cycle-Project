onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate -radix hexadecimal /testbench/dut/arm/clk
add wave -noupdate -radix hexadecimal /testbench/dut/arm/reset
add wave -noupdate -radix hexadecimal /testbench/dut/arm/Instr
add wave -noupdate -radix hexadecimal /testbench/dut/arm/c/MovF
add wave -noupdate -radix hexadecimal /testbench/dut/arm/c/MoveFlag
add wave -noupdate -radix hexadecimal -childformat {{{/testbench/dut/arm/dp/rf/rf[14]} -radix hexadecimal} {{/testbench/dut/arm/dp/rf/rf[13]} -radix hexadecimal} {{/testbench/dut/arm/dp/rf/rf[12]} -radix hexadecimal} {{/testbench/dut/arm/dp/rf/rf[11]} -radix hexadecimal} {{/testbench/dut/arm/dp/rf/rf[10]} -radix hexadecimal} {{/testbench/dut/arm/dp/rf/rf[9]} -radix hexadecimal} {{/testbench/dut/arm/dp/rf/rf[8]} -radix hexadecimal} {{/testbench/dut/arm/dp/rf/rf[7]} -radix hexadecimal} {{/testbench/dut/arm/dp/rf/rf[6]} -radix hexadecimal} {{/testbench/dut/arm/dp/rf/rf[5]} -radix hexadecimal} {{/testbench/dut/arm/dp/rf/rf[4]} -radix hexadecimal} {{/testbench/dut/arm/dp/rf/rf[3]} -radix hexadecimal} {{/testbench/dut/arm/dp/rf/rf[2]} -radix hexadecimal} {{/testbench/dut/arm/dp/rf/rf[1]} -radix hexadecimal} {{/testbench/dut/arm/dp/rf/rf[0]} -radix hexadecimal}} -expand -subitemconfig {{/testbench/dut/arm/dp/rf/rf[14]} {-radix hexadecimal} {/testbench/dut/arm/dp/rf/rf[13]} {-radix hexadecimal} {/testbench/dut/arm/dp/rf/rf[12]} {-radix hexadecimal} {/testbench/dut/arm/dp/rf/rf[11]} {-radix hexadecimal} {/testbench/dut/arm/dp/rf/rf[10]} {-radix hexadecimal} {/testbench/dut/arm/dp/rf/rf[9]} {-radix hexadecimal} {/testbench/dut/arm/dp/rf/rf[8]} {-radix hexadecimal} {/testbench/dut/arm/dp/rf/rf[7]} {-radix hexadecimal} {/testbench/dut/arm/dp/rf/rf[6]} {-radix hexadecimal} {/testbench/dut/arm/dp/rf/rf[5]} {-radix hexadecimal} {/testbench/dut/arm/dp/rf/rf[4]} {-radix hexadecimal} {/testbench/dut/arm/dp/rf/rf[3]} {-radix hexadecimal} {/testbench/dut/arm/dp/rf/rf[2]} {-radix hexadecimal} {/testbench/dut/arm/dp/rf/rf[1]} {-radix hexadecimal} {/testbench/dut/arm/dp/rf/rf[0]} {-radix hexadecimal}} /testbench/dut/arm/dp/rf/rf
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {10 ps} 0}
quietly wave cursor active 1
configure wave -namecolwidth 150
configure wave -valuecolwidth 100
configure wave -justifyvalue left
configure wave -signalnamewidth 0
configure wave -snapdistance 10
configure wave -datasetprefix 0
configure wave -rowmargin 4
configure wave -childrowmargin 2
configure wave -gridoffset 0
configure wave -gridperiod 1
configure wave -griddelta 40
configure wave -timeline 0
configure wave -timelineunits ps
update
WaveRestoreZoom {0 ps} {129 ps}
