// Copyright 2023 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Matteo Perotti <mperotti@iis.ee.ethz.ch>
// Description:
// Ara's optimized SLDU datapath.
// Cannot reshuffle AND slide at the same time.

module sldu_op_dp import ara_pkg::*; import rvv_pkg::*; import cf_math_pkg::idx_width; #(
    // Dependant parameters. DO NOT CHANGE!
    localparam int  unsigned DataWidth = $bits(elen_t), // Width of the lane datapath
    localparam int  unsigned StrbWidth = DataWidth/8,
    localparam type          strb_t    = logic [StrbWidth-1:0] // Byte-strobe type
  ) (
    input  elen_t                 op_i,
    input  logic            [2:0] slamt_i,
    input  rvv_pkg::vew_e         eew_src_i,
    input  rvv_pkg::vew_e         eew_dst_i,
    output elen_t                 op_o
);

logic [$bits(op_i)-1:0] op_i_flat;
logic [$bits(op_o)-1:0] op_o_flat;

assign op_i_flat = op_i;
assign op_o      = op_o_flat;

always_comb begin
  unique case ({eew_src_i, eew_dst_i, slamt_i})
    {EW8, EW8, 3'd0}: begin
      op_o_flat[0 +: 8] = op_i_flat[0 +: 8];
      op_o_flat[8 +: 8] = op_i_flat[8 +: 8];
      op_o_flat[16 +: 8] = op_i_flat[16 +: 8];
      op_o_flat[24 +: 8] = op_i_flat[24 +: 8];
      op_o_flat[32 +: 8] = op_i_flat[32 +: 8];
      op_o_flat[40 +: 8] = op_i_flat[40 +: 8];
      op_o_flat[48 +: 8] = op_i_flat[48 +: 8];
      op_o_flat[56 +: 8] = op_i_flat[56 +: 8];
    end
    {EW8, EW16, 3'd0}: begin
      op_o_flat[0 +: 8] = op_i_flat[0 +: 8];
      op_o_flat[8 +: 8] = op_i_flat[32 +: 8];
      op_o_flat[16 +: 8] = op_i_flat[8 +: 8];
      op_o_flat[24 +: 8] = op_i_flat[40 +: 8];
      op_o_flat[32 +: 8] = op_i_flat[16 +: 8];
      op_o_flat[40 +: 8] = op_i_flat[48 +: 8];
      op_o_flat[48 +: 8] = op_i_flat[24 +: 8];
      op_o_flat[56 +: 8] = op_i_flat[56 +: 8];
    end
    {EW8, EW32, 3'd0}: begin
      op_o_flat[0 +: 8] = op_i_flat[0 +: 8];
      op_o_flat[8 +: 8] = op_i_flat[32 +: 8];
      op_o_flat[16 +: 8] = op_i_flat[16 +: 8];
      op_o_flat[24 +: 8] = op_i_flat[48 +: 8];
      op_o_flat[32 +: 8] = op_i_flat[8 +: 8];
      op_o_flat[40 +: 8] = op_i_flat[40 +: 8];
      op_o_flat[48 +: 8] = op_i_flat[24 +: 8];
      op_o_flat[56 +: 8] = op_i_flat[56 +: 8];
    end
    {EW8, EW64, 3'd0}: begin
      op_o_flat[0 +: 8] = op_i_flat[0 +: 8];
      op_o_flat[8 +: 8] = op_i_flat[32 +: 8];
      op_o_flat[16 +: 8] = op_i_flat[16 +: 8];
      op_o_flat[24 +: 8] = op_i_flat[48 +: 8];
      op_o_flat[32 +: 8] = op_i_flat[8 +: 8];
      op_o_flat[40 +: 8] = op_i_flat[40 +: 8];
      op_o_flat[48 +: 8] = op_i_flat[24 +: 8];
      op_o_flat[56 +: 8] = op_i_flat[56 +: 8];
    end
    {EW8, EW8, 3'd1}: begin
      op_o_flat[0 +: 8] = op_i_flat[56 +:8];
      op_o_flat[8 +: 8] = op_i_flat[48 +:8];
      op_o_flat[16 +: 8] = op_i_flat[32 +:8];
      op_o_flat[24 +: 8] = op_i_flat[40 +:8];
      op_o_flat[32 +: 8] = op_i_flat[0 +:8];
      op_o_flat[40 +: 8] = op_i_flat[8 +:8];
      op_o_flat[48 +: 8] = op_i_flat[16 +:8];
      op_o_flat[56 +: 8] = op_i_flat[24 +:8];
    end
    {EW8, EW8, 3'd2}: begin
      op_o_flat[0 +: 8] = op_i_flat[24 +:8];
      op_o_flat[8 +: 8] = op_i_flat[16 +:8];
      op_o_flat[16 +: 8] = op_i_flat[0 +:8];
      op_o_flat[24 +: 8] = op_i_flat[8 +:8];
      op_o_flat[32 +: 8] = op_i_flat[56 +:8];
      op_o_flat[40 +: 8] = op_i_flat[48 +:8];
      op_o_flat[48 +: 8] = op_i_flat[32 +:8];
      op_o_flat[56 +: 8] = op_i_flat[40 +:8];
    end
    {EW8, EW8, 3'd3}: begin
      op_o_flat[0 +: 8] = op_i_flat[40 +:8];
      op_o_flat[8 +: 8] = op_i_flat[32 +:8];
      op_o_flat[16 +: 8] = op_i_flat[56 +:8];
      op_o_flat[24 +: 8] = op_i_flat[48 +:8];
      op_o_flat[32 +: 8] = op_i_flat[24 +:8];
      op_o_flat[40 +: 8] = op_i_flat[16 +:8];
      op_o_flat[48 +: 8] = op_i_flat[0 +:8];
      op_o_flat[56 +: 8] = op_i_flat[8 +:8];
    end
    {EW8, EW8, 3'd4}: begin
      op_o_flat[0 +: 8] = op_i_flat[8 +:8];
      op_o_flat[8 +: 8] = op_i_flat[0 +:8];
      op_o_flat[16 +: 8] = op_i_flat[24 +:8];
      op_o_flat[24 +: 8] = op_i_flat[16 +:8];
      op_o_flat[32 +: 8] = op_i_flat[40 +:8];
      op_o_flat[40 +: 8] = op_i_flat[32 +:8];
      op_o_flat[48 +: 8] = op_i_flat[56 +:8];
      op_o_flat[56 +: 8] = op_i_flat[48 +:8];
    end
    {EW8, EW8, 3'd5}: begin
      op_o_flat[0 +: 8] = op_i_flat[48 +:8];
      op_o_flat[8 +: 8] = op_i_flat[56 +:8];
      op_o_flat[16 +: 8] = op_i_flat[40 +:8];
      op_o_flat[24 +: 8] = op_i_flat[32 +:8];
      op_o_flat[32 +: 8] = op_i_flat[8 +:8];
      op_o_flat[40 +: 8] = op_i_flat[0 +:8];
      op_o_flat[48 +: 8] = op_i_flat[24 +:8];
      op_o_flat[56 +: 8] = op_i_flat[16 +:8];
    end
    {EW8, EW8, 3'd6}: begin
      op_o_flat[0 +: 8] = op_i_flat[16 +:8];
      op_o_flat[8 +: 8] = op_i_flat[24 +:8];
      op_o_flat[16 +: 8] = op_i_flat[8 +:8];
      op_o_flat[24 +: 8] = op_i_flat[0 +:8];
      op_o_flat[32 +: 8] = op_i_flat[48 +:8];
      op_o_flat[40 +: 8] = op_i_flat[56 +:8];
      op_o_flat[48 +: 8] = op_i_flat[40 +:8];
      op_o_flat[56 +: 8] = op_i_flat[32 +:8];
    end
    {EW8, EW8, 3'd7}: begin
      op_o_flat[0 +: 8] = op_i_flat[32 +:8];
      op_o_flat[8 +: 8] = op_i_flat[40 +:8];
      op_o_flat[16 +: 8] = op_i_flat[48 +:8];
      op_o_flat[24 +: 8] = op_i_flat[56 +:8];
      op_o_flat[32 +: 8] = op_i_flat[16 +:8];
      op_o_flat[40 +: 8] = op_i_flat[24 +:8];
      op_o_flat[48 +: 8] = op_i_flat[8 +:8];
      op_o_flat[56 +: 8] = op_i_flat[0 +:8];
    end
    {EW16, EW8, 3'd0}: begin
      op_o_flat[0 +: 8] = op_i_flat[0 +: 8];
      op_o_flat[8 +: 8] = op_i_flat[16 +: 8];
      op_o_flat[16 +: 8] = op_i_flat[32 +: 8];
      op_o_flat[24 +: 8] = op_i_flat[48 +: 8];
      op_o_flat[32 +: 8] = op_i_flat[8 +: 8];
      op_o_flat[40 +: 8] = op_i_flat[24 +: 8];
      op_o_flat[48 +: 8] = op_i_flat[40 +: 8];
      op_o_flat[56 +: 8] = op_i_flat[56 +: 8];
    end
    {EW16, EW16, 3'd0}: begin
      op_o_flat[0 +: 8] = op_i_flat[0 +: 8];
      op_o_flat[8 +: 8] = op_i_flat[8 +: 8];
      op_o_flat[16 +: 8] = op_i_flat[16 +: 8];
      op_o_flat[24 +: 8] = op_i_flat[24 +: 8];
      op_o_flat[32 +: 8] = op_i_flat[32 +: 8];
      op_o_flat[40 +: 8] = op_i_flat[40 +: 8];
      op_o_flat[48 +: 8] = op_i_flat[48 +: 8];
      op_o_flat[56 +: 8] = op_i_flat[56 +: 8];
    end
    {EW16, EW32, 3'd0}: begin
      op_o_flat[0 +: 8] = op_i_flat[0 +: 8];
      op_o_flat[8 +: 8] = op_i_flat[8 +: 8];
      op_o_flat[16 +: 8] = op_i_flat[32 +: 8];
      op_o_flat[24 +: 8] = op_i_flat[40 +: 8];
      op_o_flat[32 +: 8] = op_i_flat[16 +: 8];
      op_o_flat[40 +: 8] = op_i_flat[24 +: 8];
      op_o_flat[48 +: 8] = op_i_flat[48 +: 8];
      op_o_flat[56 +: 8] = op_i_flat[56 +: 8];
    end
    {EW16, EW64, 3'd0}: begin
      op_o_flat[0 +: 8] = op_i_flat[0 +: 8];
      op_o_flat[8 +: 8] = op_i_flat[8 +: 8];
      op_o_flat[16 +: 8] = op_i_flat[32 +: 8];
      op_o_flat[24 +: 8] = op_i_flat[40 +: 8];
      op_o_flat[32 +: 8] = op_i_flat[16 +: 8];
      op_o_flat[40 +: 8] = op_i_flat[24 +: 8];
      op_o_flat[48 +: 8] = op_i_flat[48 +: 8];
      op_o_flat[56 +: 8] = op_i_flat[56 +: 8];
    end
    {EW16, EW16, 3'd1}: begin
      op_o_flat[0 +: 8] = op_i_flat[48 +:8];
      op_o_flat[8 +: 8] = op_i_flat[56 +:8];
      op_o_flat[16 +: 8] = op_i_flat[32 +:8];
      op_o_flat[24 +: 8] = op_i_flat[40 +:8];
      op_o_flat[32 +: 8] = op_i_flat[0 +:8];
      op_o_flat[40 +: 8] = op_i_flat[8 +:8];
      op_o_flat[48 +: 8] = op_i_flat[16 +:8];
      op_o_flat[56 +: 8] = op_i_flat[24 +:8];
    end
    {EW16, EW16, 3'd2}: begin
      op_o_flat[0 +: 8] = op_i_flat[16 +:8];
      op_o_flat[8 +: 8] = op_i_flat[24 +:8];
      op_o_flat[16 +: 8] = op_i_flat[0 +:8];
      op_o_flat[24 +: 8] = op_i_flat[8 +:8];
      op_o_flat[32 +: 8] = op_i_flat[48 +:8];
      op_o_flat[40 +: 8] = op_i_flat[56 +:8];
      op_o_flat[48 +: 8] = op_i_flat[32 +:8];
      op_o_flat[56 +: 8] = op_i_flat[40 +:8];
    end
    {EW16, EW16, 3'd3}: begin
      op_o_flat[0 +: 8] = op_i_flat[32 +:8];
      op_o_flat[8 +: 8] = op_i_flat[40 +:8];
      op_o_flat[16 +: 8] = op_i_flat[48 +:8];
      op_o_flat[24 +: 8] = op_i_flat[56 +:8];
      op_o_flat[32 +: 8] = op_i_flat[16 +:8];
      op_o_flat[40 +: 8] = op_i_flat[24 +:8];
      op_o_flat[48 +: 8] = op_i_flat[0 +:8];
      op_o_flat[56 +: 8] = op_i_flat[8 +:8];
    end
    {EW32, EW8, 3'd0}: begin
      op_o_flat[0 +: 8] = op_i_flat[0 +: 8];
      op_o_flat[8 +: 8] = op_i_flat[32 +: 8];
      op_o_flat[16 +: 8] = op_i_flat[16 +: 8];
      op_o_flat[24 +: 8] = op_i_flat[48 +: 8];
      op_o_flat[32 +: 8] = op_i_flat[8 +: 8];
      op_o_flat[40 +: 8] = op_i_flat[40 +: 8];
      op_o_flat[48 +: 8] = op_i_flat[24 +: 8];
      op_o_flat[56 +: 8] = op_i_flat[56 +: 8];
    end
    {EW32, EW16, 3'd0}: begin
      op_o_flat[0 +: 8] = op_i_flat[0 +: 8];
      op_o_flat[8 +: 8] = op_i_flat[8 +: 8];
      op_o_flat[16 +: 8] = op_i_flat[32 +: 8];
      op_o_flat[24 +: 8] = op_i_flat[40 +: 8];
      op_o_flat[32 +: 8] = op_i_flat[16 +: 8];
      op_o_flat[40 +: 8] = op_i_flat[24 +: 8];
      op_o_flat[48 +: 8] = op_i_flat[48 +: 8];
      op_o_flat[56 +: 8] = op_i_flat[56 +: 8];
    end
    {EW32, EW32, 3'd0}: begin
      op_o_flat[0 +: 8] = op_i_flat[0 +: 8];
      op_o_flat[8 +: 8] = op_i_flat[8 +: 8];
      op_o_flat[16 +: 8] = op_i_flat[16 +: 8];
      op_o_flat[24 +: 8] = op_i_flat[24 +: 8];
      op_o_flat[32 +: 8] = op_i_flat[32 +: 8];
      op_o_flat[40 +: 8] = op_i_flat[40 +: 8];
      op_o_flat[48 +: 8] = op_i_flat[48 +: 8];
      op_o_flat[56 +: 8] = op_i_flat[56 +: 8];
    end
    {EW32, EW64, 3'd0}: begin
      op_o_flat[0 +: 8] = op_i_flat[0 +: 8];
      op_o_flat[8 +: 8] = op_i_flat[8 +: 8];
      op_o_flat[16 +: 8] = op_i_flat[16 +: 8];
      op_o_flat[24 +: 8] = op_i_flat[24 +: 8];
      op_o_flat[32 +: 8] = op_i_flat[32 +: 8];
      op_o_flat[40 +: 8] = op_i_flat[40 +: 8];
      op_o_flat[48 +: 8] = op_i_flat[48 +: 8];
      op_o_flat[56 +: 8] = op_i_flat[56 +: 8];
    end
    {EW32, EW32, 3'd1}: begin
      op_o_flat[0 +: 8] = op_i_flat[32 +: 8];
      op_o_flat[8 +: 8] = op_i_flat[40 +: 8];
      op_o_flat[16 +: 8] = op_i_flat[48 +: 8];
      op_o_flat[24 +: 8] = op_i_flat[56 +: 8];
      op_o_flat[32 +: 8] = op_i_flat[0 +: 8];
      op_o_flat[40 +: 8] = op_i_flat[8 +: 8];
      op_o_flat[48 +: 8] = op_i_flat[16 +: 8];
      op_o_flat[56 +: 8] = op_i_flat[24 +: 8];
    end
    {EW64, EW8, 3'd0}: begin
      op_o_flat[0 +: 8] = op_i_flat[0 +: 8];
      op_o_flat[8 +: 8] = op_i_flat[32 +: 8];
      op_o_flat[16 +: 8] = op_i_flat[16 +: 8];
      op_o_flat[24 +: 8] = op_i_flat[48 +: 8];
      op_o_flat[32 +: 8] = op_i_flat[8 +: 8];
      op_o_flat[40 +: 8] = op_i_flat[40 +: 8];
      op_o_flat[48 +: 8] = op_i_flat[24 +: 8];
      op_o_flat[56 +: 8] = op_i_flat[56 +: 8];
    end
    {EW64, EW16, 3'd0}: begin
      op_o_flat[0 +: 8] = op_i_flat[0 +: 8];
      op_o_flat[8 +: 8] = op_i_flat[8 +: 8];
      op_o_flat[16 +: 8] = op_i_flat[32 +: 8];
      op_o_flat[24 +: 8] = op_i_flat[40 +: 8];
      op_o_flat[32 +: 8] = op_i_flat[16 +: 8];
      op_o_flat[40 +: 8] = op_i_flat[24 +: 8];
      op_o_flat[48 +: 8] = op_i_flat[48 +: 8];
      op_o_flat[56 +: 8] = op_i_flat[56 +: 8];
    end
    {EW64, EW32, 3'd0}: begin
      op_o_flat[0 +: 8] = op_i_flat[0 +: 8];
      op_o_flat[8 +: 8] = op_i_flat[8 +: 8];
      op_o_flat[16 +: 8] = op_i_flat[16 +: 8];
      op_o_flat[24 +: 8] = op_i_flat[24 +: 8];
      op_o_flat[32 +: 8] = op_i_flat[32 +: 8];
      op_o_flat[40 +: 8] = op_i_flat[40 +: 8];
      op_o_flat[48 +: 8] = op_i_flat[48 +: 8];
      op_o_flat[56 +: 8] = op_i_flat[56 +: 8];
    end
    {EW64, EW64, 3'd0}: begin
      op_o_flat[0 +: 8] = op_i_flat[0 +: 8];
      op_o_flat[8 +: 8] = op_i_flat[8 +: 8];
      op_o_flat[16 +: 8] = op_i_flat[16 +: 8];
      op_o_flat[24 +: 8] = op_i_flat[24 +: 8];
      op_o_flat[32 +: 8] = op_i_flat[32 +: 8];
      op_o_flat[40 +: 8] = op_i_flat[40 +: 8];
      op_o_flat[48 +: 8] = op_i_flat[48 +: 8];
      op_o_flat[56 +: 8] = op_i_flat[56 +: 8];
    end
    {EW64, EW64, 3'd1}: begin
      op_o_flat[0 +: 8] = op_i_flat[0 +: 8];
      op_o_flat[8 +: 8] = op_i_flat[8 +: 8];
      op_o_flat[16 +: 8] = op_i_flat[16 +: 8];
      op_o_flat[24 +: 8] = op_i_flat[24 +: 8];
      op_o_flat[32 +: 8] = op_i_flat[32 +: 8];
      op_o_flat[40 +: 8] = op_i_flat[40 +: 8];
      op_o_flat[48 +: 8] = op_i_flat[48 +: 8];
      op_o_flat[56 +: 8] = op_i_flat[56 +: 8];
    end
    default: op_o_flat = op_i_flat;
  endcase
end
endmodule