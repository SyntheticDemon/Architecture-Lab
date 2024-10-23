library verilog;
use verilog.vl_types.all;
entity branch_mux is
    port(
        branch_taken    : in     vl_logic;
        branch_address  : in     vl_logic_vector(31 downto 0);
        next_pc         : in     vl_logic_vector(31 downto 0);
        pc_out          : out    vl_logic_vector(31 downto 0)
    );
end branch_mux;
