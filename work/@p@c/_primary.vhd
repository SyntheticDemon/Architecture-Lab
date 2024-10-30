library verilog;
use verilog.vl_types.all;
entity PC is
    port(
        clk             : in     vl_logic;
        reset           : in     vl_logic;
        freeze          : in     vl_logic;
        next_address    : in     vl_logic_vector(31 downto 0);
        pc              : out    vl_logic_vector(31 downto 0)
    );
end PC;
