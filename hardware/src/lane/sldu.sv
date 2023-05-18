// Copyright 2021 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Matheus Cavalcante <matheusd@iis.ee.ethz.ch>
// Description:
// This is Ara's slide unit. It is responsible for running the vector slide (up/down)
// instructions, which need access to the whole Vector Register File.

module sldu import ara_pkg::*; import rvv_pkg::*; #(
    parameter  int  unsigned NrLanes = 0,
    parameter  type          vaddr_t = logic, // Type used to address vector register file elements
    // Dependant parameters. DO NOT CHANGE!
    localparam int  unsigned DataWidth = $bits(elen_t), // Width of the lane datapath
    localparam int  unsigned StrbWidth = DataWidth/8,
    localparam type          strb_t    = logic [StrbWidth-1:0] // Byte-strobe type
  ) (
    input  logic                              clk_i,
    input  logic                              rst_ni,
    input  logic     [idx_width(NrLanes)-1:0] lane_id_i,
    // Interface with the main sequencer
    input  pe_req_t                           pe_req_i,
    input  logic                              pe_req_valid_i,
    input  logic     [NrVInsn-1:0]            pe_vinsn_running_i,
    // Interface with lane
    output pe_resp_t                          sldu_vinsn_done_o,
    input  elen_t                             sldu_operand_i,
    input  target_fu_e                        sldu_operand_target_fu_i,
    input  logic                              sldu_operand_valid_i,
    output logic                              sldu_operand_ready_o,
    output logic                              sldu_result_req_o,
    output vid_t                              sldu_result_id_o,
    output vaddr_t                            sldu_result_addr_o,
    output elen_t                             sldu_result_wdata_o,
    output strb_t                             sldu_result_be_o,
    input  logic                              sldu_result_gnt_i,
    input  logic                              sldu_result_final_gnt_i,
    // Interface with other Slide units
    input  logic                              sldu_prev_valid_i,
    input  logic                              sldu_next_valid_i,
    input  logic                              sldu_prev_ready_i,
    input  logic                              sldu_next_ready_i,
    input  elen_t                             sldu_prev_data_i,
    input  elen_t                             sldu_next_data_i,
    output logic                              sldu_prev_valid_o,
    output logic                              sldu_next_valid_o,
    output logic                              sldu_prev_ready_o,
    output logic                              sldu_next_ready_o,
    output elen_t                             sldu_prev_data_o,
    output elen_t                             sldu_next_data_o,
    // Support for reductions
    output sldu_mux_e                         sldu_mux_sel_o,
    output logic                              sldu_red_valid_o,
    // Interface with the Mask Unit
    input  strb_t                             mask_i,
    input  logic                              mask_valid_i,
    output logic                              mask_ready_o
  );

  `include "common_cells/registers.svh"

  import cf_math_pkg::idx_width;

  ////////////////////////////////
  //  Vector instruction queue  //
  ////////////////////////////////

  // We store a certain number of in-flight vector instructions
  localparam VInsnQueueDepth = SlduInsnQueueDepth;

  struct packed {
    pe_req_t [VInsnQueueDepth-1:0] vinsn;

    // Each instruction can be in one of the three execution phases.
    // - Being accepted (i.e., it is being stored for future execution in this
    //   vector functional unit).
    // - Being issued (i.e., its micro-operations are currently being issued
    //   to the corresponding functional units).
    // - Being committed (i.e., its results are being written to the vector
    //   register file).
    // We need pointers to index which instruction is at each execution phase
    // between the VInsnQueueDepth instructions in memory.
    logic [idx_width(VInsnQueueDepth)-1:0] accept_pnt;
    logic [idx_width(VInsnQueueDepth)-1:0] issue_pnt;
    logic [idx_width(VInsnQueueDepth)-1:0] commit_pnt;

    // We also need to count how many instructions are queueing to be
    // issued/committed, to avoid accepting more instructions than
    // we can handle.
    logic [idx_width(VInsnQueueDepth):0] issue_cnt;
    logic [idx_width(VInsnQueueDepth):0] commit_cnt;
  } vinsn_queue_d, vinsn_queue_q;

  pe_req_t vinsn_issue_q;
  logic vinsn_issue_valid_q;
  // Is the vector instruction queue full?
  logic vinsn_queue_full;
  assign vinsn_queue_full = (vinsn_queue_q.commit_cnt == VInsnQueueDepth);

  // Do we have a vector instruction ready to be issued?
  `FF(vinsn_issue_q, vinsn_queue_d.vinsn[vinsn_queue_d.issue_pnt], '0)
  `FF(vinsn_issue_valid_q, vinsn_queue_d.issue_cnt != '0, 1'b0)

  // Do we have a vector instruction with results being committed?
  pe_req_t vinsn_commit;
  logic    vinsn_commit_valid;
  assign vinsn_commit       = vinsn_queue_q.vinsn[vinsn_queue_q.commit_pnt];
  assign vinsn_commit_valid = (vinsn_queue_q.commit_cnt != '0);

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      vinsn_queue_q <= '0;
    end else begin
      vinsn_queue_q <= vinsn_queue_d;
    end
  end

  /////////////////////
  //  Result queues  //
  /////////////////////

  localparam int unsigned ResultQueueDepth = 4;

  // There is a result queue per lane, holding the results that were not
  // yet accepted by the corresponding lane.
  typedef struct packed {
    vid_t id;
    vaddr_t addr;
    elen_t wdata;
    strb_t be;
  } payload_t;

  // Result queue
  payload_t [ResultQueueDepth-1:0]            result_queue_d, result_queue_q;
  logic     [ResultQueueDepth-1:0]            result_queue_valid_d, result_queue_valid_q;
  // We need two pointers in the result queue. One pointer to
  // indicate with `payload_t` we are currently writing into (write_pnt),
  // and one pointer to indicate which `payload_t` we are currently
  // reading from and writing into the lanes (read_pnt).
  logic     [idx_width(ResultQueueDepth)-1:0] result_queue_write_pnt_d, result_queue_write_pnt_q;
  logic     [idx_width(ResultQueueDepth)-1:0] result_queue_read_pnt_d, result_queue_read_pnt_q;
  // We need to count how many valid elements are there in this result queue.
  logic     [idx_width(ResultQueueDepth):0]   result_queue_cnt_d, result_queue_cnt_q;
  // Vector to register the final grants from the operand requesters, which indicate
  // that the result was actually written in the VRF (while the normal grant just says
  // that the result was accepted by the operand requester stage
  logic                                       result_final_gnt_d, result_final_gnt_q;

  // Is the result queue full?
  logic result_queue_full;
  assign result_queue_full = (result_queue_cnt_q == ResultQueueDepth);
  // Is the result queue empty?
  logic result_queue_empty;
  assign result_queue_empty = (result_queue_cnt_q == '0);

  always_ff @(posedge clk_i or negedge rst_ni) begin: p_result_queue_ff
    if (!rst_ni) begin
      result_queue_q           <= '0;
      result_queue_valid_q     <= '0;
      result_queue_write_pnt_q <= '0;
      result_queue_read_pnt_q  <= '0;
      result_queue_cnt_q       <= '0;
    end else begin
      result_queue_q           <= result_queue_d;
      result_queue_valid_q     <= result_queue_valid_d;
      result_queue_write_pnt_q <= result_queue_write_pnt_d;
      result_queue_read_pnt_q  <= result_queue_read_pnt_d;
      result_queue_cnt_q       <= result_queue_cnt_d;
    end
  end

  ////////////////////////////////
  //  Spill-reg from the lanes  //
  ////////////////////////////////

  // elen_t      sldu_operand;
  // logic       sldu_operand_valid;
  // logic       sldu_operand_ready;
  // target_fu_e sldu_operand_target_fu_d, sldu_operand_target_fu_q;

  // // Don't handshake if the operands target the addrgen!
  // // Moreover, when computing NP2 slides, loop over the same data!
  // elen_t sldu_operand_d;
  // logic  sldu_operand_valid_d;
  // logic  sldu_operand_ready_q;

  // typedef enum logic {
  //   NP2_BUFFER_PNT,
  //   NP2_RESULT_PNT
  // } np2_result_queue_pnt_e;

  // typedef enum logic {
  //   NP2_EXT_SEL,
  //   NP2_LOOP_SEL
  // } np2_loop_mux_e;
  // np2_loop_mux_e np2_loop_mux_sel_d, np2_loop_mux_sel_q;

  // logic slide_np2_buf_valid_d, slide_np2_buf_valid_q;


  // spill_register #(
  //   .T(elen_t)
  // ) i_sldu_spill_register (
  //   .clk_i  (clk_i                   ),
  //   .rst_ni (rst_ni                  ),
  //   .valid_i(sldu_operand_valid_d    ),
  //   .ready_o(sldu_operand_ready_q    ),
  //   .data_i (sldu_operand_d          ),
  //   .valid_o(sldu_operand_valid      ),
  //   .ready_i(sldu_operand_ready      ),
  //   .data_o (sldu_operand            )
  // );

  // assign sldu_operand_d = np2_loop_mux_sel_q == NP2_EXT_SEL
  //                        ? sldu_operand_i
  //                        : result_queue_q[NP2_BUFFER_PNT].wdata;

  // assign sldu_operand_valid_d = (sldu_operand_target_fu_q == ALU_SLDU)
  //                              ? (np2_loop_mux_sel_q == NP2_EXT_SEL
  //                                ? sldu_operand_valid_i
  //                                : slide_np2_buf_valid_q)
  //                              : 1'b0;

  // assign sldu_operand_ready_o = (sldu_operand_target_fu_q == ALU_SLDU)
  //                              ? (np2_loop_mux_sel_q == NP2_EXT_SEL
  //                                ? sldu_operand_ready_q
  //                                : 1'b0)
  //                              : 1'b0;

  // assign sldu_operand_target_fu_d = sldu_operand_target_fu_i;

  // always_ff @(posedge clk_i or negedge rst_ni) begin
  //   if (!rst_ni)
  //     sldu_operand_target_fu_q <= target_fu_e'(1'b0);
  //   else
  //     sldu_operand_target_fu_q <= sldu_operand_target_fu_d;
  // end

  //////////////////////////
  //  Cut from the masku  //
  //////////////////////////

  // strb_t mask_new;
  // logic  mask_valid_d, mask_valid_q;
  logic  mask_ready_d;
  logic  mask_ready_q;
  logic mask_valid;

  // stream_register #(
  //   .T(strb_t)
  // ) i_mask_operand_register (
  //   .clk_i     (clk_i        ),
  //   .rst_ni    (rst_ni       ),
  //   .clr_i     (1'b0         ),
  //   .testmode_i(1'b0         ),
  //   .data_o    (mask_new     ),
  //   .valid_o   (mask_valid_q ),
  //   .ready_i   (mask_ready_d ),
  //   .data_i    (mask_i       ),
  //   .valid_i   (mask_valid_d ),
  //   .ready_o   (mask_ready_q )
  // );

  // Sample only SLDU mask valid
  assign mask_valid = mask_valid_i & ~vinsn_issue_q.vm & vinsn_issue_valid_q;
  

  // Don't upset the masku with a spurious ready
  assign mask_ready_o = mask_ready_q & mask_valid_i & ~vinsn_issue_q.vm & vinsn_issue_valid_q & !(vinsn_issue_q.vfu inside {VFU_Alu, VFU_MFpu});

  //////////////////
  //  Mask queue  //
  //////////////////

  strb_t [3:0] mask_d, mask_q;
  // First mask at mask_pnt, second mask one after, becomes first mask after write back
  logic  [1:0] mask_pnt_d, mask_pnt_q;
  logic  [3:0] mask_queue_valid_d, mask_queue_valid_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if(~rst_ni) begin
      mask_q             <= '0;
      mask_pnt_q         <= '0;
      mask_queue_valid_q <= '0;
      mask_ready_q       <= '0;
    end else begin
      mask_q             <= mask_d;
      mask_pnt_q         <= mask_pnt_d;
      mask_queue_valid_q <= mask_queue_valid_d;
      mask_ready_q       <= mask_ready_d;
    end
  end

  ///////////////////
  //  NP2 Support  //
  ///////////////////

  // The SLDU only supports powers of two (p2) strides
  // Decompose the non-power-of-two (np2) slide in multiple p2 slides

  // We implement the np2 support here and fully process every input packet
  // singularly to comply with the undisturbed policy. We cannot use the VRF
  // as intermediate buffer; each VRF write is a commit.

  typedef logic [idx_width(8*NrLanes)-1:0] stride_t;

  // stride_t                                  p2_stride_gen_stride_d;
  // logic                                     p2_stride_gen_valid_d;
  // logic                                     p2_stride_gen_update_d;
  // logic [idx_width(idx_width(8*NrLanes)):0] p2_stride_gen_popc_q;
  // stride_t                                  p2_stride_gen_stride_q;
  // logic                                     p2_stride_gen_valid_q;

  // p2_stride_gen #(
  //   .NrLanes (NrLanes)
  // ) i_p2_stride_gen (
  //   .clk_i       (clk_i                 ),
  //   .rst_ni      (rst_ni                ),
  //   .stride_i    (p2_stride_gen_stride_d),
  //   .valid_i     (p2_stride_gen_valid_d ),
  //   .update_i    (p2_stride_gen_update_d),
  //   .popc_o      (p2_stride_gen_popc_q  ),
  //   .stride_p2_o (p2_stride_gen_stride_q),
  //   .valid_o     (p2_stride_gen_valid_q )
  // );

  //////////////////
  //  Reductions  //
  //////////////////

  // Inter-lane reductions are performed with a logarithmic tree, and the result is
  // accumulated in the last Lane. Then, in the end, the result is passed to the first
  // lane for SIMD reduction
  // logic [idx_width(NrLanes)-1:0] red_stride_cnt_d, red_stride_cnt_q;
  // logic [idx_width(NrLanes):0] red_stride_cnt_d_wide;

  // logic is_issue_reduction, is_issue_alu_reduction, is_issue_vmfpu_reduction;

  // assign is_issue_alu_reduction   = vinsn_issue_valid_q & (vinsn_issue_q.vfu == VFU_Alu);
  // assign is_issue_vmfpu_reduction = vinsn_issue_valid_q & (vinsn_issue_q.vfu == VFU_MFpu);
  // assign is_issue_reduction       = is_issue_alu_reduction | is_issue_vmfpu_reduction;

  always_comb begin
    sldu_mux_sel_o = NO_RED;
    // if ((is_issue_alu_reduction && !(vinsn_commit_valid && vinsn_commit.vfu != VFU_Alu)) || (vinsn_commit_valid && vinsn_commit.vfu == VFU_Alu)) begin
    //   sldu_mux_sel_o = ALU_RED;
    // end else if ((is_issue_vmfpu_reduction && !(vinsn_commit_valid && vinsn_commit.vfu != VFU_MFpu)) || (vinsn_commit_valid && vinsn_commit.vfu == VFU_MFpu)) begin
    //   sldu_mux_sel_o = MFPU_RED;
    // end
  end


  ///////////////////
  //  Slide queue  //
  ///////////////////

  localparam int unsigned SlideQueueDepth = 2;

  // Slide queue
  elen_t slide_queue_in, slide_queue_out;
  logic  slide_queue_push, slide_queue_pop, slide_queue_empty, slide_queue_full;

  fifo_v3 #(
    .DATA_WIDTH (ELEN             ),
    .DEPTH      (SlideQueueDepth  )
  ) i_slide_queue (
    .clk_i      (clk_i            ),
    .rst_ni     (rst_ni           ),
    .flush_i    (/* Unused */     ),
    .testmode_i (/* Unused */     ),
    .full_o     (slide_queue_full ),
    .empty_o    (slide_queue_empty),
    .usage_o    (/* Unused */     ),
    .data_i     (slide_queue_in   ),
    .push_i     (slide_queue_push ),
    .data_o     (slide_queue_out  ),
    .pop_i      (slide_queue_pop  )
  );


  /////////////////////
  //  SLDU DataPath  //
  /////////////////////

  // Input/output non-flat operands
  elen_t slide_fin;
  elen_t shuffle_in, shuffle_out;
  elen_t slide_result;

  // Input and output eew for reshuffling
  rvv_pkg::vew_e sld_eew_src;
  rvv_pkg::vew_e sld_eew_dst;

  // 0: slidedown, 1: slideup
  logic sld_dir;

  // The SLDU slides by powers of two
  stride_t sld_slamt;
  logic   [idx_width(StrbWidth)-1:0] shuffle_amt;

  // Max Slide: NrLanes/2, Min Slide: 0
  logic [idx_width(NrLanes)-1:0] amt_d, amt_q, slide_cnt_d, slide_cnt_q;
  // 0: to prev, 1: to next
  logic dir_d, dir_q;

  sldu_op_dp #(
  ) i_sldu_op_dp (
    .op_i     (shuffle_in ),
    .slamt_i  (shuffle_amt),
    .eew_src_i(sld_eew_src),
    .eew_dst_i(sld_eew_dst),
    .op_o     (shuffle_out)
  );


  //////////////////
  //  Slide unit  //
  //////////////////

  // Interface with the main sequencer
  pe_resp_t pe_resp;

  // Vector instructions currently running
  logic [NrVInsn-1:0] vinsn_running_d, vinsn_running_q;

  typedef enum logic [1:0] {
    SLIDE_IDLE,
    SLIDE_NO_SLIDE,
    SLIDE_RUN,
    SLIDE_LAST
  } slide_reg_state_e;
  slide_reg_state_e slide_state_d, slide_state_q;

  // Remaining bytes of the current instruction in the issue phase
  vlen_t issue_cnt_d, issue_cnt_q;
  vlen_t mask_cnt_d, mask_cnt_q;
  logic  first_slide_d, first_slide_q;
  logic  first_mask_d, first_mask_q;

  // Remaining bytes of the current instruction in the commit phase
  vlen_t commit_cnt_d, commit_cnt_q;

  logic  [7:0] out_en_flat1, out_en_seq1, out_en_flat2, out_en_seq2;
  strb_t       out_en1, out_en2;
  
  // Pointers in the input operand and the output result
  logic   [idx_width(NrLanes*StrbWidth):0] in_pnt_d, in_pnt_q;
  logic   [idx_width(StrbWidth):0] out_pnt_d, out_pnt_q;
  vaddr_t                                  vrf_pnt_d, vrf_pnt_q;

  // Respected by default: input_limit_d  = 8*NrLanes + out_pnt_d - in_pnt_d;
  // To enforce: output_limit_d = out_pnt_d + issue_cnt_d;
  logic [idx_width(MAXVL+1):0] output_limit_d, output_limit_q;

  logic sldu_valid_d, sldu_valid_q;
  logic sldu_ready_d, sldu_ready_q;
  logic slide_op_valid, slide_fin_valid, slide_fin_valid_next_cycle;
  elen_t slide_op;

  always_comb begin
    // Maintain state
    slide_state_d          = slide_state_q;
    slide_cnt_d            = slide_cnt_q;
    issue_cnt_d            = issue_cnt_q;
    first_slide_d          = first_slide_q;
    first_mask_d           = first_mask_q;
    amt_d                  = amt_q;
    dir_d                  = dir_q;
    commit_cnt_d           = commit_cnt_q;
    vinsn_queue_d          = vinsn_queue_q;
    result_final_gnt_d     = result_final_gnt_q;
    sldu_valid_d           = sldu_valid_q;
    sldu_ready_d           = sldu_ready_q;
    vrf_pnt_d              = vrf_pnt_q;
    output_limit_d         = output_limit_q;

    mask_d                 = mask_q;
    mask_pnt_d             = mask_pnt_q;
    mask_cnt_d             = mask_cnt_q;
    mask_queue_valid_d     = mask_queue_valid_q;

    result_queue_d           = result_queue_q;
    result_queue_valid_d     = result_queue_valid_q;
    result_queue_read_pnt_d  = result_queue_read_pnt_q;
    result_queue_write_pnt_d = result_queue_write_pnt_q;
    result_queue_cnt_d       = result_queue_cnt_q;

    slide_fin_valid        = 1'b0;
    slide_queue_push       = 1'b0;
    slide_queue_pop        = 1'b0;
    sldu_operand_ready_o   = 1'b0;
    mask_ready_d           = 1'b0;
    pe_resp                = '0;
    out_en_flat1           = '0;
    out_en_seq1            = '0;
    out_en1                = '0;
    out_en_flat2           = '0;
    out_en_seq2            = '0;
    out_en2                = '0;

    // Vector instructions currently running
    vinsn_running_d = vinsn_running_q & pe_vinsn_running_i;

    sld_eew_src = (vinsn_issue_q.vfu inside {VFU_Alu, VFU_MFpu})
                ? vinsn_issue_q.vtype.vsew
                : vinsn_issue_q.eew_vs2;
    sld_eew_dst = vinsn_issue_q.vtype.vsew;
    sld_dir     = (vinsn_issue_q.op == VSLIDEUP) || (vinsn_issue_q.vfu inside {VFU_Alu, VFU_MFpu});
    // sld_slamt   = (vinsn_issue_q.vfu inside {VFU_Alu, VFU_MFpu})
    //             ? red_stride_cnt_q
    //             : stride_t'(vinsn_issue_q.stride >> vinsn_issue_q.vtype.vsew);
    sld_slamt   = stride_t'(vinsn_issue_q.stride >> vinsn_issue_q.vtype.vsew);
    // shuffle_amt = vinsn_issue_q.op == VSLIDEUP ? int'(out_pnt_q >> vinsn_issue_q.vtype.vsew)
    //                                            : 8 - int'(out_pnt_q >> vinsn_issue_q.vtype.vsew);
    shuffle_amt = int'(out_pnt_q >> vinsn_issue_q.vtype.vsew);

    // New operand from lane op queue
    slide_op_valid = sldu_operand_target_fu_i == ALU_SLDU ? sldu_operand_valid_i : 1'b0;          // may need a pipeline register
    slide_op       = sldu_operand_i;                                                              // same

    // Outputs
    sldu_prev_valid_o = sldu_valid_q;
    sldu_next_valid_o = sldu_valid_q;
    sldu_prev_ready_o = sldu_ready_q;
    sldu_next_ready_o = sldu_ready_q;
    sldu_prev_data_o  = slide_queue_out;
    sldu_next_data_o  = slide_queue_out;

    /////////////////
    //  Slide FSM  //
    /////////////////

    unique case (slide_state_q)
      SLIDE_IDLE: begin
        if (slide_op_valid && vinsn_issue_valid_q && slide_queue_empty) begin

          // Calculate slide amount and direction from instruction
          // amt means to which lane the element needs to go and dir is wether we slide to next or prev lane
          // amt_d = sld_slamt[idx_width(NrLanes)-1:0] == (NrLanes/2) ? NrLanes/2 & sld_slamt[idx_width(NrLanes)-1:0]:
          //                                                     sld_slamt[idx_width(NrLanes)-1:0];                                 // doesn't work for 1 or 2 lanes
          // dir_d = vinsn_issue_q.op == VSLIDEDOWN ? sld_slamt[idx_width(NrLanes)-2] :
          //                           sld_slamt[idx_width(NrLanes)-1] ^ 1'b1;
          vrf_pnt_d = '0;


          unique case (vinsn_issue_q.op)
            VSLIDEUP: begin
              automatic int slamt = sld_slamt - 1;
              amt_d = NrLanes - sld_slamt[idx_width(NrLanes)-1:0];
              // vslideup starts reading the source operand from its beginning
              in_pnt_d  = '0;

              // vslideup starts writing the destination vector at the slide offset
              //out_pnt_d = lane_id_i < sld_slamt ? vinsn_issue_q.stride[idx_width(8*NrLanes)-1:0] : '0;
              // out_pnt_d = vinsn_issue_q.stride[idx_width(8*NrLanes)-1:idx_width(NrLanes)];
              // out_pnt_d = vinsn_issue_q.stride[idx_width(8*NrLanes)-1:0];
              // if (int'(sld_slamt[idx_width(NrLanes)-1:0]) <= lane_id_i)
              //   out_pnt_d -= int'(1 << vinsn_issue_q.vtype.vsew);
              out_pnt_d = slamt[idx_width(NrLanes)+3:idx_width(NrLanes)];
              if (lane_id_i <= slamt[idx_width(NrLanes)-1:0])
                out_pnt_d += 1;
              out_pnt_d = out_pnt_d << vinsn_issue_q.vtype.vsew;

              // Initialize counters
              issue_cnt_d = vinsn_issue_q.vl << int'(vinsn_issue_q.vtype.vsew);
              issue_cnt_d = issue_cnt_d >> $clog2(NrLanes);
              mask_cnt_d  = issue_cnt_d;

              // Initialize be-enable-generation ancillary signals
              // output_limit_d = vinsn_issue_q.use_scalar_op ? out_pnt_d + issue_cnt_d/NrLanes : issue_cnt_d/NrLanes;
              output_limit_d = '1;

              // Trim vector elements which are not touched by the slide unit
              // issue_cnt_d -= vinsn_issue_q.stride[$bits(issue_cnt_d)-1:0];

              // Start writing at the middle of the destination vector
              vrf_pnt_d = vinsn_issue_q.stride >> $clog2(8*NrLanes);

              // Only needed for vslide1up to write the first element
              // first_slide_d = vinsn_issue_q.use_scalar_op && lane_id_i == 0 ? 1'b1 : 1'b0;
              if (vinsn_issue_q.use_scalar_op && lane_id_i == 0) begin
                first_slide_d = 1'b1;
                // if (vinsn_issue_q.vtype.vsew == EW64)
                //   issue_cnt_d += 8;
              end else begin
                first_slide_d = 1'b0;
              end
            end
            VSLIDEDOWN: begin
              automatic int last_line = vinsn_issue_q.vl[idx_width(NrLanes)-1:0] - 1;
              automatic int slamt = sld_slamt;
              amt_d = sld_slamt[idx_width(NrLanes)-1:0];
              // vslidedown starts reading the source operand from the slide offset
              in_pnt_d  = vinsn_issue_q.stride[idx_width(8*NrLanes)-1:0];
              // vslidedown starts writing the destination vector at its beginning
              // out_pnt_d = 8 - slamt[idx_width(NrLanes)+3:idx_width(NrLanes)];
              // if (lane_id_i + 1 >= NrLanes - slamt[idx_width(NrLanes)-1:0])
              //   out_pnt_d -= 1;
              // out_pnt_d = out_pnt_d << vinsn_issue_q.vtype.vsew;
              out_pnt_d = 8 - int'(slamt[idx_width(NrLanes)+3:idx_width(NrLanes)] << vinsn_issue_q.vtype.vsew);
              if (sld_slamt[idx_width(NrLanes)-1:0] >= NrLanes - lane_id_i)
                out_pnt_d -= int'(1 << vinsn_issue_q.vtype.vsew);

              // Initialize counters
              issue_cnt_d = vinsn_issue_q.vl << int'(vinsn_issue_q.vtype.vsew);
              issue_cnt_d = issue_cnt_d >> $clog2(NrLanes);
              output_limit_d = issue_cnt_d;
              if (vinsn_issue_q.use_scalar_op && (last_line[idx_width(NrLanes)-1:0] == lane_id_i))
                output_limit_d -= 1 << int'(vinsn_issue_q.vtype.vsew);
              issue_cnt_d += out_pnt_d;
              mask_cnt_d  = issue_cnt_d;

              first_slide_d = 1'b1;

              // Initialize be-enable-generation ancillary signals
              // output_limit_d = vinsn_issue_q.use_scalar_op
              //                ? issue_cnt_d - (1 << int'(vinsn_issue_q.vtype.vsew))
              //                : issue_cnt_d;

              // Trim the last element of vslide1down, which does not come from the VRF
              // if (vinsn_issue_q.use_scalar_op)
              //   issue_cnt_d -= 1 << int'(vinsn_issue_q.vtype.vsew);
            end
          endcase

          if (amt_d == 0) begin
            slide_state_d = SLIDE_NO_SLIDE;
          end else if (amt_d == 1 || (NrLanes - amt_d) == 1) begin
            slide_state_d = SLIDE_LAST;
            // Load the first operand into the slide queue
            sldu_operand_ready_o = 1'b1;                                           // may need register
            slide_cnt_d = '0;
            slide_queue_in = slide_op;
            slide_queue_push = 1'b1;
            sldu_valid_d = 1'b1;
          end else begin
            slide_state_d = SLIDE_RUN;
            // Load the first operand into the slide queue
            sldu_operand_ready_o = 1'b1;                                           // may need register
            slide_cnt_d = '0;
            slide_queue_in = slide_op;
            slide_queue_push = 1'b1;
            sldu_valid_d = 1'b1;
          end

          if (~vinsn_issue_q.vm)
            first_mask_d = 1'b1;
        end
      end
      SLIDE_NO_SLIDE: begin
        if (slide_op_valid && vinsn_issue_valid_q && ~slide_queue_full && (vinsn_issue_q.vm || (mask_queue_valid_q[mask_pnt_q] && (mask_queue_valid_q[mask_pnt_q + 1] || issue_cnt_q < 9)))) begin
          slide_fin = slide_op;
          slide_fin_valid = 1'b1;
          sldu_operand_ready_o = 1'b1;                                             // may need register
          if (issue_cnt_q <= 8 || output_limit_q <= out_pnt_q) begin
            slide_state_d = SLIDE_IDLE;
          end
        end else if (issue_cnt_q <= 8 || output_limit_q <= out_pnt_q) begin
            slide_state_d = SLIDE_IDLE;
        end

        // // No slide needed? Operand direclty goes into the shuffle stage
        // if (amt_q == 0) begin                                    // all slides with amt=0 should be handled by this
        //   slide_fin = slide_op;
        //   slide_fin_valid = 1'b1;                                //ins finish? SLIDE_IDLE

        // // Slide gets started
        // end else begin                                           // TODO: lookahed handshake -> direct slide without waiting 1 cycle
        //   slide_cnt_d = '0;
        //   slide_queue_in = slide_op;
        //   slide_queue_push = 1'b1;
        //   sldu_valid_d = 1'b1;

        //   slide_state_d = (amt_q == 1) ? SLIDE_LAST : SLIDE_RUN;
        // end
      end
      SLIDE_RUN: begin
        // Slide to next lane
        if (amt_q[idx_width(NrLanes)-1]) begin
          // Set the output to the next lane valid
          if (sldu_next_valid_o) begin
            slide_queue_pop = 1'b1;
            sldu_valid_d = 1'b0;
          end
          // Get the operand from the prev lane
          if (sldu_prev_valid_i) begin
            slide_queue_in = sldu_prev_data_i;
            slide_queue_push = 1'b1;
            slide_cnt_d += 1;
            sldu_valid_d = 1'b1;
            if (slide_cnt_q + 2 == amt_q) begin
              slide_state_d = SLIDE_LAST;
            end
          end
        // Slide to prev lane
        end else begin
          // Set the output to the prev lane valid
          if(sldu_prev_valid_o) begin
            slide_queue_pop = 1'b1;
            sldu_valid_d = 1'b0;
          end
          // Get the operand from the next lane
          if(sldu_next_valid_i) begin
            slide_queue_in = sldu_next_data_i;
            slide_queue_push = 1'b1;
            slide_cnt_d += 1;
            sldu_valid_d = 1'b1;
            if (slide_cnt_q + 2 == amt_q) begin
              slide_state_d = SLIDE_LAST;
            end
          end
        end
      end
      SLIDE_LAST: begin
        if (vinsn_issue_q.vm || (mask_queue_valid_q[mask_pnt_q] && (mask_queue_valid_q[mask_pnt_q + 1] || issue_cnt_q < 9))) begin
          if (amt_q[idx_width(NrLanes)-1] && sldu_prev_valid_i) begin
            // Slide from prev lane and finish
            slide_fin = sldu_prev_data_i;
            slide_fin_valid = 1'b1;
          end else if (~amt_q[idx_width(NrLanes)-1] && sldu_next_valid_i) begin
            // Slide from next lane and finish
            slide_fin = sldu_next_data_i;
            slide_fin_valid = 1'b1;
          end
          // Pop the element if the other lane received it
          if (amt_q[idx_width(NrLanes)-1] && sldu_next_valid_o) begin
            slide_queue_pop = 1'b1;
            sldu_valid_d = 1'b0;
          end else if (~amt_q[idx_width(NrLanes)-1] && sldu_prev_valid_o) begin
            slide_queue_pop = 1'b1;
            sldu_valid_d = 1'b0;
          end
          // Start a new slide
          if (slide_op_valid && vinsn_issue_valid_q && ~slide_queue_full && output_limit_q > 8) begin
            sldu_operand_ready_o = 1'b1;                                           // may need register
            slide_cnt_d = '0;
            slide_queue_in = slide_op;
            slide_queue_push = 1'b1;
            sldu_valid_d = 1'b1;

            slide_state_d = (amt_d == 1 || (NrLanes - amt_d) == 1) ? SLIDE_LAST : SLIDE_RUN;
          end else if (issue_cnt_q <= 8 || output_limit_q <= 8) begin
            slide_state_d = SLIDE_IDLE;                                            // just for testing could be to soon if next op isnt ready
          end
        end
      end
    endcase

    //////////////////////
    //  Shuffle & Mask  //
    //////////////////////

    shuffle_in = slide_fin;

    if (sld_dir) begin
      slide_result = lane_id_i < sld_slamt ? shuffle_out : shuffle_in;
    end else begin
      slide_result = NrLanes-1 - lane_id_i < sld_slamt ? shuffle_out : shuffle_in;
    end

    if (~vinsn_issue_q.vm && mask_valid) begin
      if (first_mask_q) begin
        mask_d[mask_pnt_q] = mask_i;
        mask_queue_valid_d[mask_pnt_q] = 1'b1;
        first_mask_d = 1'b0;
        mask_cnt_d = mask_cnt_q > 7 ? mask_cnt_q - 8 : '0;
        mask_ready_d = mask_cnt_q > 0 ? 1'b1 : 1'b0;
      end else if (mask_cnt_q > 0 && !mask_queue_valid_d[mask_pnt_q + 1]) begin
        mask_d[mask_pnt_q + 1] = mask_i;
        mask_queue_valid_d[mask_pnt_q + 1] = 1'b1;
        mask_cnt_d = mask_cnt_q > 7 ? mask_cnt_q - 8 : '0;
        mask_ready_d = mask_cnt_q > 0 ? 1'b1 : 1'b0;
      end
    end

    /////////////////////////////
    //  Write to result queue  //
    /////////////////////////////

    if (slide_fin_valid) begin
      if (vinsn_issue_q.op == VSLIDEUP) begin
        // If this is a vslide1up instruction, copy the scalar operand to the first word
        if (first_slide_q) begin
          first_slide_d = 1'b0;
          unique case (vinsn_issue_q.vtype.vsew)
            EW8: begin
              result_queue_d[result_queue_write_pnt_q].wdata[7:0] =
                vinsn_issue_q.scalar_op[7:0];
              result_queue_d[result_queue_write_pnt_q].be[0:0] =
                vinsn_issue_q.vm || mask_q[mask_pnt_q][0];
            end
            EW16: begin
              result_queue_d[result_queue_write_pnt_q].wdata[15:0] =
                vinsn_issue_q.scalar_op[15:0];
              result_queue_d[result_queue_write_pnt_q].be[1:0] =
                {2{vinsn_issue_q.vm || mask_q[mask_pnt_q][0]}};
            end
            EW32: begin
              result_queue_d[result_queue_write_pnt_q].wdata[31:0] =
                vinsn_issue_q.scalar_op[31:0];
              result_queue_d[result_queue_write_pnt_q].be[3:0] =
                {4{vinsn_issue_q.vm || mask_q[mask_pnt_q][0]}};
            end
            EW64: begin
              result_queue_d[result_queue_write_pnt_q].wdata[63:0] =
                vinsn_issue_q.scalar_op[63:0];
              result_queue_d[result_queue_write_pnt_q].be[7:0] =
                {8{vinsn_issue_q.vm || mask_q[mask_pnt_q][0]}};
            end
          endcase
        end
        // Build the sequential byte-output-enable
        for (int unsigned b = 0; b < 8; b++)
          if ((b >= out_pnt_q && b < issue_cnt_q) || vinsn_issue_q.vfu inside {VFU_Alu, VFU_MFpu})
            out_en_seq1[b] = 1'b1;

        // Shuffle the output enable
        for (int unsigned b = 0; b < 8; b++)
          out_en_flat1[shuffle_index(b, 1, vinsn_issue_q.vtype.vsew)] = out_en_seq1[b];

        // Mask the output enable with the mask vector
        out_en1 = out_en_flat1 & ({8{vinsn_issue_q.vm | (vinsn_issue_q.vfu inside {VFU_Alu, VFU_MFpu})}} | mask_q[mask_pnt_q]);

        // Write in the correct bytes
        for (int b = 0; b < 8; b++)
          if (out_en1[b]) begin
            result_queue_d[result_queue_write_pnt_q].wdata[8*b +: 8] = slide_result[8*b +: 8];
            result_queue_d[result_queue_write_pnt_q].be[b]           = 1'b1;
          end

        // Initialize id and addr fields of the result queue requests
        result_queue_d[result_queue_write_pnt_q].id   = vinsn_issue_q.id;
        result_queue_d[result_queue_write_pnt_q].addr = vaddr(vinsn_issue_q.vd, NrLanes) + vrf_pnt_q;

        issue_cnt_d = issue_cnt_q < 8 ? '0 : issue_cnt_q - 8;
        mask_queue_valid_d[mask_pnt_q] = 1'b0;
        mask_pnt_d  = mask_pnt_q + 1;

        // Increment VRF address
        vrf_pnt_d = vrf_pnt_q + 1;

        // Send result to the VRF
        result_queue_cnt_d += 1;
        result_queue_valid_d[result_queue_write_pnt_q] = '1;
        result_queue_write_pnt_d                       = result_queue_write_pnt_q + 1;
        if (result_queue_write_pnt_q == ResultQueueDepth-1)
          result_queue_write_pnt_d = '0;

        // Build the sequential byte-output-enable
        for (int unsigned b = 0; b < 8; b++)
          if ((b < out_pnt_q && b < issue_cnt_d) || vinsn_issue_q.vfu inside {VFU_Alu, VFU_MFpu})
            out_en_seq2[b] = 1'b1;

        // Shuffle the output enable
        for (int unsigned b = 0; b < 8; b++)
          out_en_flat2[shuffle_index(b, 1, vinsn_issue_q.vtype.vsew)] = out_en_seq2[b];

        // Mask the output enable with the mask vector
        out_en2 = out_en_flat2 & ({8*NrLanes{vinsn_issue_q.vm | (vinsn_issue_q.vfu inside {VFU_Alu, VFU_MFpu})}} | mask_q[mask_pnt_d]);

        // Write in the correct bytes
        for (int b = 0; b < 8; b++)
          if (out_en2[b]) begin
            result_queue_d[result_queue_write_pnt_d].wdata[8*b +: 8] = slide_result[8*b +: 8];
            result_queue_d[result_queue_write_pnt_d].be[b]           = 1'b1;
          end

        // Finished the operation
        if (issue_cnt_q <= 8) begin
          // Reset the logarighmic counter
          // red_stride_cnt_d = 1;

          // Increment vector instruction queue pointers and counters
          vinsn_queue_d.issue_pnt += 1;
          vinsn_queue_d.issue_cnt -= 1;
        end
      end else if (vinsn_issue_q.op == VSLIDEDOWN) begin
        if (first_slide_q) begin
          first_slide_d = 1'b0;
        end else begin
          // Build the sequential byte-output-enable
          for (int unsigned b = 0; b < 8; b++)
            if ((b >= out_pnt_q && b < output_limit_q && b < issue_cnt_q) || vinsn_issue_q.vfu inside {VFU_Alu, VFU_MFpu})
              out_en_seq1[b] = 1'b1;

          // Shuffle the output enable
          for (int unsigned b = 0; b < 8; b++)
            out_en_flat1[shuffle_index(b, 1, vinsn_issue_q.vtype.vsew)] = out_en_seq1[b];

          // Mask the output enable with the mask vector
          out_en1 = out_en_flat1 & ({8{vinsn_issue_q.vm | (vinsn_issue_q.vfu inside {VFU_Alu, VFU_MFpu})}} | mask_q[mask_pnt_q]);

          // Write in the correct bytes
          for (int b = 0; b < 8; b++)
            if (out_en1[b]) begin
              result_queue_d[result_queue_write_pnt_q].wdata[8*b +: 8] = slide_result[8*b +: 8];
              result_queue_d[result_queue_write_pnt_q].be[b]           = 1'b1;
            end

          issue_cnt_d = issue_cnt_q < 8 ? '0 : issue_cnt_q - 8;
          output_limit_d = output_limit_q < 8 ? '0 : output_limit_q - 8;
          mask_queue_valid_d[mask_pnt_q] = 1'b0;
          mask_pnt_d  = mask_pnt_q + 1;

          // Increment VRF address
          vrf_pnt_d = vrf_pnt_q + 1;

          // Send result to the VRF
          result_queue_cnt_d += 1;
          result_queue_valid_d[result_queue_write_pnt_q] = '1;
          result_queue_write_pnt_d                       = result_queue_write_pnt_q + 1;
          if (result_queue_write_pnt_q == ResultQueueDepth-1)
            result_queue_write_pnt_d = '0;
        end

        // Initialize id and addr fields of the result queue requests
        result_queue_d[result_queue_write_pnt_q].id   = vinsn_issue_q.id;
        result_queue_d[result_queue_write_pnt_q].addr = vaddr(vinsn_issue_q.vd, NrLanes) + vrf_pnt_q;

        // Build the sequential byte-output-enable
        for (int unsigned b = 0; b < 8; b++)
          if ((b < out_pnt_q && b < output_limit_d && b < issue_cnt_d) || vinsn_issue_q.vfu inside {VFU_Alu, VFU_MFpu})
            out_en_seq2[b] = 1'b1;

        // Shuffle the output enable
        for (int unsigned b = 0; b < 8; b++)
          out_en_flat2[shuffle_index(b, 1, vinsn_issue_q.vtype.vsew)] = out_en_seq2[b];

        // Mask the output enable with the mask vector
        out_en2 = out_en_flat2 & ({8*NrLanes{vinsn_issue_q.vm | (vinsn_issue_q.vfu inside {VFU_Alu, VFU_MFpu})}} | mask_q[mask_pnt_d]);

        // Write in the correct bytes
        for (int b = 0; b < 8; b++)
          if (out_en2[b]) begin
            result_queue_d[result_queue_write_pnt_d].wdata[8*b +: 8] = slide_result[8*b +: 8];
            result_queue_d[result_queue_write_pnt_d].be[b]           = 1'b1;
          end

        // if (first_slide_q) begin
        //   output_limit_d = output_limit_q > out_pnt_q ? output_limit_q - out_pnt_q : '0;
        // end

        // Finished the operation
        if (output_limit_q <= 8 || ( output_limit_q <= out_pnt_q)) begin//first_slide_q &&
          automatic int last_line = vinsn_issue_q.vl[idx_width(NrLanes)-1:0] - 1;
          // If this is a vslide1down, fill up the last position with the scalar operand
          if (vinsn_issue_q.use_scalar_op && (last_line[idx_width(NrLanes)-1:0] == lane_id_i)) begin
            // Copy the scalar operand to the last word
            automatic int out_seq_byte = output_limit_q;
            automatic int out_byte = shuffle_index(out_seq_byte, 1, vinsn_issue_q.vtype.vsew);
            // automatic int tgt_lane = out_byte[3 +: $clog2(NrLanes)];
            automatic int tgt_lane_offset = out_byte[2:0];

            unique case (vinsn_issue_q.vtype.vsew)
              EW8: begin
                result_queue_d[result_queue_write_pnt_q].wdata[8*out_byte +: 8]
                  = vinsn_issue_q.scalar_op[7:0];
                result_queue_d[result_queue_write_pnt_q].be[out_byte +: 1] =
                  vinsn_issue_q.vm || mask_q[mask_pnt_q][out_byte];
              end
              EW16: begin
                result_queue_d[result_queue_write_pnt_q].wdata[8*out_byte +: 16]
                  = vinsn_issue_q.scalar_op[15:0];
                result_queue_d[result_queue_write_pnt_q].be[out_byte +: 2] =
                  {2{vinsn_issue_q.vm || mask_q[mask_pnt_q][out_byte]}};
              end
              EW32: begin
                result_queue_d[result_queue_write_pnt_q].wdata[8*out_byte +: 32]
                  = vinsn_issue_q.scalar_op[31:0];
                result_queue_d[result_queue_write_pnt_q].be[out_byte +: 4] =
                  {4{vinsn_issue_q.vm || mask_q[mask_pnt_q][out_byte]}};
              end
              EW64: begin
                result_queue_d[result_queue_write_pnt_q].wdata[8*out_byte +: 64]
                  = vinsn_issue_q.scalar_op[63:0];
                result_queue_d[result_queue_write_pnt_q].be[out_byte +: 8] =
                  {8{vinsn_issue_q.vm || mask_q[mask_pnt_q][out_byte]}};
              end
            endcase
          end
          // Reset the logarighmic counter
          // red_stride_cnt_d = 1;
          // Send result to the VRF
          result_queue_cnt_d += 1;
          result_queue_valid_d[result_queue_write_pnt_q] = '1;
          if (first_slide_q) begin
            result_queue_write_pnt_d                       = result_queue_write_pnt_q + 1;
            if (result_queue_write_pnt_q == ResultQueueDepth-1)
              result_queue_write_pnt_d = '0;
          end

          // Increment vector instruction queue pointers and counters
          vinsn_queue_d.issue_pnt += 1;
          vinsn_queue_d.issue_cnt -= 1;
        end
      end
    end

    // NEW FOR VSLIDEDOWN

    // if (slide_fin_valid || slide_fin_valid_next_cycle) begin
    // // if (slide_fin_valid || (slide_fin_valid_next_cycle && issue_cnt_q > 7)) begin
    //   // How many bytes are we copying from the operand to the destination, in this cycle?
    //   automatic int in_byte_count = 8 - in_pnt_q;
    //   automatic int out_byte_count = 8 - out_pnt_q;
    //   automatic int byte_count = in_byte_count < out_byte_count ? in_byte_count : out_byte_count;

    //   // Build the sequential byte-output-enable
    //   for (int unsigned b = 0; b < 8; b++)
    //     if ((b >= out_pnt_q && b < issue_cnt_q) || vinsn_issue_q.vfu inside {VFU_Alu, VFU_MFpu})
    //       out_en_seq[b] = 1'b1;

    //   // // Build the sequential byte-output-enable
    //   // for (int unsigned b = 0; b < 8; b++)
    //   //   if ((((b >= out_pnt_q && slide_fin_valid) || (b < out_pnt_q && slide_fin_valid_next_cycle)) && b < output_limit_q) || vinsn_issue_q.vfu inside {VFU_Alu, VFU_MFpu})
    //   //     out_en_seq[b] = 1'b1;

    //   // Shuffle the output enable
    //   for (int unsigned b = 0; b < 8; b++)
    //     out_en_flat[shuffle_index(b, 1, vinsn_issue_q.vtype.vsew)] = out_en_seq[b];

    //   // Mask the output enable with the mask vector
    //   out_en = out_en_flat & ({8*NrLanes{vinsn_issue_q.vm | (vinsn_issue_q.vfu inside {VFU_Alu, VFU_MFpu})}} | mask_q);

    //   // Write in the correct bytes
    //   for (int b = 0; b < 8; b++)
    //     if (out_en[b]) begin
    //       result_queue_d[result_queue_write_pnt_q].wdata[8*b +: 8] = slide_result[8*b +: 8];
    //       result_queue_d[result_queue_write_pnt_q].be[b]           = 1'b1;
    //     end

    //   // Initialize id and addr fields of the result queue requests
    //   result_queue_d[result_queue_write_pnt_q].id   = vinsn_issue_q.id;
    //   result_queue_d[result_queue_write_pnt_q].addr = vaddr(vinsn_issue_q.vd, NrLanes) + vrf_pnt_q;

    //   // Bump pointers (reductions always finish in one shot)
    //   in_pnt_d    = vinsn_issue_q.vfu inside {VFU_Alu, VFU_MFpu} ? 8                  : in_pnt_q  + byte_count;
    //   out_pnt_d   = vinsn_issue_q.vfu inside {VFU_Alu, VFU_MFpu} ? 8                  : out_pnt_q + byte_count;
    //   issue_cnt_d = vinsn_issue_q.vfu inside {VFU_Alu, VFU_MFpu} ? issue_cnt_q - 8    : issue_cnt_q - byte_count;
    //   // issue_cnt_d = (vinsn_issue_q.vfu inside {VFU_Alu, VFU_MFpu} || slide_fin_valid_next_cycle) ? issue_cnt_q -8 : issue_cnt_q;

    //   // Read a full word from the VRF or finished the instruction
    //   if (in_pnt_d == 8 || issue_cnt_q <= byte_count) begin
    //     // Reset the pointer
    //     in_pnt_d = '0;
    //   end

    //   // Filled up a word to the VRF or finished the instruction
    //   if (out_pnt_d == 8 || issue_cnt_q <= byte_count) begin
    //     // Reset the pointer
    //     // out_pnt_d = vinsn_issue_q.vfu inside {VFU_Alu, VFU_MFpu} ? {'0, red_stride_cnt_d, 3'b0} : '0;
    //     out_pnt_d = '0;
    //     // We used all the bits of the mask
    //     if (vinsn_issue_q.op inside {VSLIDEUP, VSLIDEDOWN})
    //       mask_ready_d = !vinsn_issue_q.vm;

    //     // Increment VRF address
    //     vrf_pnt_d = vrf_pnt_q + 1;

    //     // Send result to the VRF
    //     result_queue_cnt_d += 1;
    //     result_queue_valid_d[result_queue_write_pnt_q] = '1;
    //     result_queue_write_pnt_d                       = result_queue_write_pnt_q + 1;
    //     if (result_queue_write_pnt_q == ResultQueueDepth-1)
    //       result_queue_write_pnt_d = '0;
    //   end

    //   // Finished the operation
    //   if (issue_cnt_q <= byte_count || (vinsn_issue_q.vfu inside {VFU_Alu, VFU_MFpu} && issue_cnt_q <= 8)) begin
    //     // Reset the logarighmic counter
    //     // red_stride_cnt_d = 1;

    //     // Increment vector instruction queue pointers and counters
    //     vinsn_queue_d.issue_pnt += 1;
    //     vinsn_queue_d.issue_cnt -= 1;
    //   end

    //   // If this is a vslide1up instruction, copy the scalar operand to the first word
    //   if (first_slide_q) begin
    //     first_slide_d = 1'b0;
    //     unique case (vinsn_issue_q.vtype.vsew)
    //       EW8: begin
    //         result_queue_d[result_queue_write_pnt_q].wdata[7:0] =
    //           vinsn_issue_q.scalar_op[7:0];
    //         result_queue_d[result_queue_write_pnt_q].be[0:0] =
    //           vinsn_issue_q.vm || mask_q[0][0];
    //       end
    //       EW16: begin
    //         result_queue_d[result_queue_write_pnt_q].wdata[15:0] =
    //           vinsn_issue_q.scalar_op[15:0];
    //         result_queue_d[result_queue_write_pnt_q].be[1:0] =
    //           {2{vinsn_issue_q.vm || mask_q[0][0]}};
    //       end
    //       EW32: begin
    //         result_queue_d[result_queue_write_pnt_q].wdata[31:0] =
    //           vinsn_issue_q.scalar_op[31:0];
    //         result_queue_d[result_queue_write_pnt_q].be[3:0] =
    //           {4{vinsn_issue_q.vm || mask_q[0][0]}};
    //       end
    //       EW64: begin
    //         result_queue_d[result_queue_write_pnt_q].wdata[63:0] =
    //           vinsn_issue_q.scalar_op[63:0];
    //         result_queue_d[result_queue_write_pnt_q].be[7:0] =
    //           {8{vinsn_issue_q.vm || mask_q[0][0]}};
    //         // issue_cnt_d += 8;
    //       end
    //     endcase
    //   end

    //   // If this is a vslide1down, fill up the last position with the scalar operand
    //   if (vinsn_issue_q.op == VSLIDEDOWN && vinsn_issue_q.use_scalar_op && (issue_cnt_q == lane_id_i)) begin
    //     // Copy the scalar operand to the last word
    //     automatic int out_seq_byte = issue_cnt_q;
    //     automatic int out_byte = shuffle_index(out_seq_byte, 1, vinsn_issue_q.vtype.vsew);
    //     automatic int tgt_lane = out_byte[3 +: $clog2(NrLanes)];
    //     automatic int tgt_lane_offset = out_byte[2:0];

    //     unique case (vinsn_issue_q.vtype.vsew)
    //       EW8: begin
    //         result_queue_d[result_queue_write_pnt_q].wdata[8*tgt_lane_offset +: 8]
    //           = vinsn_issue_q.scalar_op[7:0];
    //         result_queue_d[result_queue_write_pnt_q].be[tgt_lane_offset +: 1] =
    //           vinsn_issue_q.vm || mask_q[tgt_lane][tgt_lane_offset];
    //       end
    //       EW16: begin
    //         result_queue_d[result_queue_write_pnt_q].wdata[8*tgt_lane_offset +: 16]
    //           = vinsn_issue_q.scalar_op[15:0];
    //         result_queue_d[result_queue_write_pnt_q].be[tgt_lane_offset +: 2] =
    //           {2{vinsn_issue_q.vm || mask_q[tgt_lane][tgt_lane_offset]}};
    //       end
    //       EW32: begin
    //         result_queue_d[result_queue_write_pnt_q].wdata[8*tgt_lane_offset +: 32]
    //           = vinsn_issue_q.scalar_op[31:0];
    //         result_queue_d[result_queue_write_pnt_q].be[tgt_lane_offset +: 4] =
    //           {4{vinsn_issue_q.vm || mask_q[tgt_lane][tgt_lane_offset]}};
    //       end
    //       EW64: begin
    //         result_queue_d[result_queue_write_pnt_q].wdata[8*tgt_lane_offset +: 64]
    //           = vinsn_issue_q.scalar_op[63:0];
    //         result_queue_d[result_queue_write_pnt_q].be[tgt_lane_offset +: 8] =
    //           {8{vinsn_issue_q.vm || mask_q[tgt_lane][tgt_lane_offset]}};
    //       end
    //     endcase
    //   end
    // end

  //////////////////
  //  Slide unit  //
  //////////////////

  // // Vector instructions currently running
  // logic [NrVInsn-1:0] vinsn_running_d, vinsn_running_q;

  // // Interface with the main sequencer
  // pe_resp_t pe_resp;

  // // State of the slide FSM
  // typedef enum logic [3:0] {
  //   SLIDE_IDLE,
  //   SLIDE_RUN,
  //   SLIDE_RUN_VSLIDE1UP_FIRST_WORD,
  //   SLIDE_RUN_OSUM,
  //   SLIDE_WAIT_OSUM,
  //   SLIDE_NP2_SETUP,
  //   SLIDE_NP2_RUN,
  //   SLIDE_NP2_COMMIT,
  //   SLIDE_NP2_WAIT
  // } slide_state_e;
  // slide_state_e state_d, state_q;

  // logic  [8*NrLanes-1:0] out_en_flat, out_en_seq;
  // strb_t [NrLanes-1:0]   out_en;

  // // Pointers in the input operand and the output result
  // logic   [idx_width(NrLanes*StrbWidth):0] in_pnt_d, in_pnt_q;
  // logic   [idx_width(NrLanes*StrbWidth):0] out_pnt_d, out_pnt_q;
  // vaddr_t                                  vrf_pnt_d, vrf_pnt_q;

  // // Remaining bytes of the current instruction in the issue phase
  // vlen_t issue_cnt_d, issue_cnt_q;
  // // Respected by default: input_limit_d  = 8*NrLanes + out_pnt_d - in_pnt_d;
  // // To enforce: output_limit_d = out_pnt_d + issue_cnt_d;
  // logic [idx_width(MAXVL+1):0] output_limit_d, output_limit_q;

  // // Remaining bytes of the current instruction in the commit phase
  // vlen_t commit_cnt_d, commit_cnt_q;

  // always_comb begin: p_sldu
  //   // Maintain state
  //   vinsn_queue_d = vinsn_queue_q;
  //   issue_cnt_d   = issue_cnt_q;
  //   commit_cnt_d  = commit_cnt_q;
  //   in_pnt_d      = in_pnt_q;
  //   out_pnt_d     = out_pnt_q;
  //   vrf_pnt_d     = vrf_pnt_q;
  //   state_d       = state_q;

  //   result_queue_d           = result_queue_q;
  //   result_queue_valid_d     = result_queue_valid_q;
  //   result_queue_read_pnt_d  = result_queue_read_pnt_q;
  //   result_queue_write_pnt_d = result_queue_write_pnt_q;
  //   result_queue_cnt_d       = result_queue_cnt_q;

  //   result_final_gnt_d = result_final_gnt_q;

  //   // Vector instructions currently running
  //   vinsn_running_d = vinsn_running_q & pe_vinsn_running_i;

  //   out_en_flat    = '0;
  //   out_en_seq     = '0;
  //   out_en         = '0;
  //   output_limit_d = output_limit_q;

  //   // We are not ready, by default
  //   pe_resp            = '0;
  //   mask_ready_d       = 1'b0;
  //   sldu_operand_ready = '0;

  //   red_stride_cnt_d = red_stride_cnt_q;

  //   p2_stride_gen_stride_d = '0;
  //   p2_stride_gen_valid_d  = 1'b0;
  //   p2_stride_gen_update_d = 1'b0;

  //   np2_loop_mux_sel_d    = np2_loop_mux_sel_q;
  //   slide_np2_buf_valid_d = slide_np2_buf_valid_q;

  //   red_stride_cnt_d_wide = {red_stride_cnt_q, red_stride_cnt_q[idx_width(NrLanes)-1]};

  //   // Inform the main sequencer if we are idle
  //   pe_req_ready_o = !vinsn_queue_full;

  //   // Slide Unit DP
  //   sld_op_src  = sldu_operand;
  //   sld_eew_src = (vinsn_issue_q.vfu inside {VFU_Alu, VFU_MFpu})
  //               ? vinsn_issue_q.vtype.vsew
  //               : vinsn_issue_q.eew_vs2;
  //   sld_eew_dst = vinsn_issue_q.vtype.vsew;
  //   sld_dir     = (vinsn_issue_q.op == VSLIDEUP) || (vinsn_issue_q.vfu inside {VFU_Alu, VFU_MFpu});
  //   sld_slamt   = (vinsn_issue_q.vfu inside {VFU_Alu, VFU_MFpu})
  //               ? red_stride_cnt_q
  //               : stride_t'(vinsn_issue_q.stride >> vinsn_issue_q.vtype.vsew);

  //   /////////////////
  //   //  Slide FSM  //
  //   /////////////////

  //   unique case (state_q)
  //     SLIDE_IDLE: begin
  //       if (vinsn_issue_valid_q) begin
  //         state_d   = vinsn_issue_q.is_stride_np2 ? SLIDE_NP2_SETUP : SLIDE_RUN;
  //         vrf_pnt_d = '0;

  //         unique case (vinsn_issue_q.op)
  //           VSLIDEUP: begin
  //             // vslideup starts reading the source operand from its beginning
  //             in_pnt_d  = '0;
  //             // vslideup starts writing the destination vector at the slide offset
  //             out_pnt_d = vinsn_issue_q.stride[idx_width(8*NrLanes)-1:0];

  //             // Initialize counters
  //             issue_cnt_d = vinsn_issue_q.vl << int'(vinsn_issue_q.vtype.vsew);

  //             // Initialize be-enable-generation ancillary signals
  //             output_limit_d = vinsn_issue_q.use_scalar_op ? out_pnt_d + issue_cnt_d : issue_cnt_d;

  //             // Trim vector elements which are not touched by the slide unit
  //             issue_cnt_d -= vinsn_issue_q.stride[$bits(issue_cnt_d)-1:0];

  //             // Start writing at the middle of the destination vector
  //             vrf_pnt_d = vinsn_issue_q.stride >> $clog2(8*NrLanes);

  //             // Go to SLIDE_RUN_VSLIDE1UP_FIRST_WORD if this is a vslide1up instruction
  //             if (vinsn_issue_q.use_scalar_op)
  //               state_d = SLIDE_RUN_VSLIDE1UP_FIRST_WORD;
  //           end
  //           VSLIDEDOWN: begin
  //             // vslidedown starts reading the source operand from the slide offset
  //             in_pnt_d  = vinsn_issue_q.stride[idx_width(8*NrLanes)-1:0];
  //             // vslidedown starts writing the destination vector at its beginning
  //             out_pnt_d = '0;

  //             // Initialize counters
  //             issue_cnt_d = vinsn_issue_q.vl << int'(vinsn_issue_q.vtype.vsew);

  //             // Initialize be-enable-generation ancillary signals
  //             output_limit_d = vinsn_issue_q.use_scalar_op
  //                            ? issue_cnt_d - (1 << int'(vinsn_issue_q.vtype.vsew))
  //                            : issue_cnt_d;

  //             // Trim the last element of vslide1down, which does not come from the VRF
  //             if (vinsn_issue_q.use_scalar_op)
  //               issue_cnt_d -= 1 << int'(vinsn_issue_q.vtype.vsew);
  //           end
  //           // Ordered sum reductions
  //           VFREDOSUM, VFWREDOSUM: begin
  //             // Ordered redsum instructions doesn't need in/out_pnt
  //             in_pnt_d  = '0;
  //             out_pnt_d = '0;

  //             // The total number of transactions is vl - 1, but the last data is sent
  //             // to lane 0
  //             issue_cnt_d  = vinsn_issue_q.vl;

  //             state_d = SLIDE_RUN_OSUM;
  //           end
  //           // Unordered reductions
  //           default: begin
  //             // Unordered redsum instructions doesn't need in/out_pnt
  //             in_pnt_d  = '0;
  //             out_pnt_d = '0;

  //             // Initialize issue cnt. Pretend to move NrLanes 64-bit elements for (clog2(NrLanes) + 1) times.
  //             issue_cnt_d  = (NrLanes * ($clog2(NrLanes) + 1)) << EW64;
  //           end
  //         endcase
  //       end
  //     end

  //     SLIDE_RUN, SLIDE_RUN_VSLIDE1UP_FIRST_WORD, SLIDE_NP2_COMMIT: begin
  //       // Are we ready?
  //       // During a reduction (vinsn_issue_q.vfu == VFU_Alu/VFU_MFPU) don't wait for mask bits
  //       if ((&sldu_operand_valid ||
  //          (((vinsn_issue_q.stride[$bits(vinsn_issue_q.vl)-1:0] >> vinsn_issue_q.vtype.vsew) >= vinsn_issue_q.vl) &&
  //          (state_q == SLIDE_RUN_VSLIDE1UP_FIRST_WORD))) &&
  //          !result_queue_full && (vinsn_issue_q.vm || vinsn_issue_q.vfu inside {VFU_Alu, VFU_MFpu} || (|mask_valid_q)))
  //       begin

  //         // How many bytes are we copying from the operand to the destination, in this cycle?
  //         automatic int in_byte_count = NrLanes * 8 - in_pnt_q;
  //         automatic int out_byte_count = NrLanes * 8 - out_pnt_q;
  //         automatic int byte_count = in_byte_count < out_byte_count ? in_byte_count : out_byte_count;

  //         // Build the sequential byte-output-enable
  //         for (int unsigned b = 0; b < 8*NrLanes; b++)
  //           if ((b >= out_pnt_q && b < output_limit_q) || vinsn_issue_q.vfu inside {VFU_Alu, VFU_MFpu})
  //             out_en_seq[b] = 1'b1;

  //         // Shuffle the output enable
  //         for (int unsigned b = 0; b < 8*NrLanes; b++)
  //           out_en_flat[shuffle_index(b, NrLanes, vinsn_issue_q.vtype.vsew)] = out_en_seq[b];

  //         // Mask the output enable with the mask vector
  //         out_en = out_en_flat & ({8*NrLanes{vinsn_issue_q.vm | (vinsn_issue_q.vfu inside {VFU_Alu, VFU_MFpu})}} | mask_q);

  //         // Write in the correct bytes
  //         for (int lane = 0; lane < NrLanes; lane++)
  //           for (int b = 0; b < 8; b++)
  //             if (out_en[lane][b]) begin
  //               result_queue_d[result_queue_write_pnt_q][lane].wdata[8*b +: 8] = sld_op_dst[lane][8*b +: 8];
  //               result_queue_d[result_queue_write_pnt_q][lane].be[b]           = 1'b1;
  //             end

  //         // Initialize id and addr fields of the result queue requests
  //         for (int lane = 0; lane < NrLanes; lane++) begin
  //           result_queue_d[result_queue_write_pnt_q][lane].id   = vinsn_issue_q.id;
  //           result_queue_d[result_queue_write_pnt_q][lane].addr =
  //             vaddr(vinsn_issue_q.vd, NrLanes) + vrf_pnt_q;
  //         end

  //         // Bump pointers (reductions always finish in one shot)
  //         in_pnt_d    = vinsn_issue_q.vfu inside {VFU_Alu, VFU_MFpu} ? NrLanes * 8                  : in_pnt_q  + byte_count;
  //         out_pnt_d   = vinsn_issue_q.vfu inside {VFU_Alu, VFU_MFpu} ? NrLanes * 8                  : out_pnt_q + byte_count;
  //         issue_cnt_d = vinsn_issue_q.vfu inside {VFU_Alu, VFU_MFpu} ? issue_cnt_q - (NrLanes * 8)  : issue_cnt_q - byte_count;

  //         // In Jump to SLIDE_RUN if stride is P2
  //         if (state_q != SLIDE_NP2_COMMIT)
  //           state_d = SLIDE_RUN;

  //         // If this is a vslide1up instruction, copy the scalar operand to the first word
  //         if (state_q == SLIDE_RUN_VSLIDE1UP_FIRST_WORD)
  //           unique case (vinsn_issue_q.vtype.vsew)
  //             EW8: begin
  //               result_queue_d[result_queue_write_pnt_q][0].wdata[7:0] =
  //                 vinsn_issue_q.scalar_op[7:0];
  //               result_queue_d[result_queue_write_pnt_q][0].be[0:0] =
  //                 vinsn_issue_q.vm || mask_q[0][0];
  //             end
  //             EW16: begin
  //               result_queue_d[result_queue_write_pnt_q][0].wdata[15:0] =
  //                 vinsn_issue_q.scalar_op[15:0];
  //               result_queue_d[result_queue_write_pnt_q][0].be[1:0] =
  //                 {2{vinsn_issue_q.vm || mask_q[0][0]}};
  //             end
  //             EW32: begin
  //               result_queue_d[result_queue_write_pnt_q][0].wdata[31:0] =
  //                 vinsn_issue_q.scalar_op[31:0];
  //               result_queue_d[result_queue_write_pnt_q][0].be[3:0] =
  //                 {4{vinsn_issue_q.vm || mask_q[0][0]}};
  //             end
  //             EW64: begin
  //               result_queue_d[result_queue_write_pnt_q][0].wdata[63:0] =
  //                 vinsn_issue_q.scalar_op[63:0];
  //               result_queue_d[result_queue_write_pnt_q][0].be[7:0] =
  //                 {8{vinsn_issue_q.vm || mask_q[0][0]}};
  //             end
  //           endcase

  //         // Read a full word from the VRF or finished the instruction
  //         if (in_pnt_d == NrLanes * 8 || issue_cnt_q <= byte_count) begin
  //           // Reset the pointer and ask for a new operand
  //           in_pnt_d           = '0;
  //           sldu_operand_ready = '1;
  //           // Left-rotate the logarithmic counter. Hacky way to write it, but it's to
  //           // deal with the 2-lanes design without complaints from Verilator...
  //           // wide signal to please the tool
  //           red_stride_cnt_d_wide = {red_stride_cnt_q, red_stride_cnt_q[idx_width(NrLanes)-1]};
  //           red_stride_cnt_d      = red_stride_cnt_d_wide[idx_width(NrLanes)-1:0];

  //           // Reset the input data mux
  //           np2_loop_mux_sel_d = NP2_EXT_SEL;
  //           if (state_q == SLIDE_NP2_COMMIT) begin
  //             // Jump to NP2 setup again
  //             state_d = SLIDE_NP2_SETUP;
  //           end
  //         end

  //         // Filled up a word to the VRF or finished the instruction
  //         if (out_pnt_d == NrLanes * 8 || issue_cnt_q <= byte_count) begin
  //           // Reset the pointer
  //           out_pnt_d = vinsn_issue_q.vfu inside {VFU_Alu, VFU_MFpu} ? {'0, red_stride_cnt_d, 3'b0} : '0;
  //           // We used all the bits of the mask
  //           if (vinsn_issue_q.op inside {VSLIDEUP, VSLIDEDOWN})
  //             mask_ready_d = !vinsn_issue_q.vm;

  //           // Increment VRF address
  //           vrf_pnt_d = vrf_pnt_q + 1;

  //           // Send result to the VRF
  //           result_queue_cnt_d += 1;
  //           result_queue_valid_d[result_queue_write_pnt_q] = '1;
  //           result_queue_write_pnt_d                       = result_queue_write_pnt_q + 1;
  //           if (result_queue_write_pnt_q == ResultQueueDepth-1)
  //             result_queue_write_pnt_d = '0;

  //           if (state_q == SLIDE_NP2_COMMIT) state_d = SLIDE_NP2_WAIT;
  //         end

  //         // Finished the operation
  //         if (issue_cnt_q <= byte_count || (vinsn_issue_q.vfu inside {VFU_Alu, VFU_MFpu} && issue_cnt_q <= 8 * NrLanes)) begin
  //           // Back to idle
  //           state_d = SLIDE_IDLE;
  //           // Reset the logarighmic counter
  //           red_stride_cnt_d = 1;

  //           // If this is a vslide1down, fill up the last position with the scalar operand
  //           if (vinsn_issue_q.op == VSLIDEDOWN && vinsn_issue_q.use_scalar_op) begin
  //             // Copy the scalar operand to the last word
  //             automatic int out_seq_byte = issue_cnt_q;
  //             automatic int out_byte = shuffle_index(out_seq_byte, NrLanes, vinsn_issue_q.vtype.vsew);
  //             automatic int tgt_lane = out_byte[3 +: $clog2(NrLanes)];
  //             automatic int tgt_lane_offset = out_byte[2:0];

  //             unique case (vinsn_issue_q.vtype.vsew)
  //               EW8: begin
  //                 result_queue_d[result_queue_write_pnt_q][tgt_lane].wdata[8*tgt_lane_offset +: 8]
  //                   = vinsn_issue_q.scalar_op[7:0];
  //                 result_queue_d[result_queue_write_pnt_q][tgt_lane].be[tgt_lane_offset +: 1] =
  //                   vinsn_issue_q.vm || mask_q[tgt_lane][tgt_lane_offset];
  //               end
  //               EW16: begin
  //                 result_queue_d[result_queue_write_pnt_q][tgt_lane].wdata[8*tgt_lane_offset +: 16]
  //                   = vinsn_issue_q.scalar_op[15:0];
  //                 result_queue_d[result_queue_write_pnt_q][tgt_lane].be[tgt_lane_offset +: 2] =
  //                   {2{vinsn_issue_q.vm || mask_q[tgt_lane][tgt_lane_offset]}};
  //               end
  //               EW32: begin
  //                 result_queue_d[result_queue_write_pnt_q][tgt_lane].wdata[8*tgt_lane_offset +: 32]
  //                   = vinsn_issue_q.scalar_op[31:0];
  //                 result_queue_d[result_queue_write_pnt_q][tgt_lane].be[tgt_lane_offset +: 4] =
  //                   {4{vinsn_issue_q.vm || mask_q[tgt_lane][tgt_lane_offset]}};
  //               end
  //               EW64: begin
  //                 result_queue_d[result_queue_write_pnt_q][tgt_lane].wdata[8*tgt_lane_offset +: 64]
  //                   = vinsn_issue_q.scalar_op[63:0];
  //                 result_queue_d[result_queue_write_pnt_q][tgt_lane].be[tgt_lane_offset +: 8] =
  //                   {8{vinsn_issue_q.vm || mask_q[tgt_lane][tgt_lane_offset]}};
  //               end
  //             endcase
  //           end

  //           // Increment vector instruction queue pointers and counters
  //           vinsn_queue_d.issue_pnt += 1;
  //           vinsn_queue_d.issue_cnt -= 1;
  //         end
  //       end
  //       if (state_q == SLIDE_NP2_COMMIT)
  //         if (slide_np2_buf_valid_q && sldu_operand_ready_q[0])
  //           // Reset the buffer-valid if the buffer is read, by default
  //           slide_np2_buf_valid_d = 1'b0;
  //     end
  //     SLIDE_RUN_OSUM: begin
  //       // Short Note: For ordered sum reduction instruction, only one lane has a valid data, and it is sent to the next lane
  //       // Don't wait for mask bits
  //       if (!result_queue_full) begin
  //         for (int lane = 0; lane < NrLanes; lane++) begin
  //           if (sldu_operand_valid[lane]) begin
  //             automatic int tgt_lane = (lane == NrLanes - 1) ? 0 : lane + 1;
  //             // Send result to lane 0
  //             if (issue_cnt_q == 1) tgt_lane = 0;

  //             // Acknowledge the received operand
  //             sldu_operand_ready[lane] = 1'b1;

  //             // Send result to next lane
  //             result_queue_d[result_queue_write_pnt_q][tgt_lane].wdata =
  //               sldu_operand[lane];
  //             result_queue_d[result_queue_write_pnt_q][tgt_lane].be =
  //               {8{vinsn_issue_q.vm}} | mask_q[tgt_lane];
  //             result_queue_valid_d[result_queue_write_pnt_q][tgt_lane] = '1;

  //             issue_cnt_d = issue_cnt_q - 1;
  //           end
  //         end
  //       end

  //       // Finish the operation
  //       if (issue_cnt_d == '0) begin
  //         state_d      = SLIDE_WAIT_OSUM;
  //         // Increment vector instruction queue pointers and counters
  //         vinsn_queue_d.issue_pnt += 1;
  //         vinsn_queue_d.issue_cnt -= 1;
  //       end
  //     end
  //     SLIDE_WAIT_OSUM: begin
  //       // Wait one cycle for the last result processing
  //       commit_cnt_d = 1'b0;
  //       state_d      = SLIDE_IDLE;
  //     end
  //     SLIDE_NP2_SETUP: begin
  //       // Prepare the write pointer
  //       result_queue_write_pnt_d = NP2_BUFFER_PNT;
  //       // Prepare the read pointer
  //       result_queue_read_pnt_d = NP2_RESULT_PNT;
  //       // Setup the mux sel as soon as we get one operand
  //       if (sldu_operand_valid_i[0])
  //         np2_loop_mux_sel_d = NP2_LOOP_SEL;
  //       // Setup the p2-stride generator
  //       p2_stride_gen_stride_d = stride_t'(vinsn_issue_q.stride >> vinsn_issue_q.vtype.vsew);
  //       p2_stride_gen_valid_d  = 1'b1;
  //       // Start processing the first VRF chunk as soon as the result queue is completely empty
  //       if (np2_loop_mux_sel_q == NP2_LOOP_SEL && result_queue_empty) begin
  //         state_d = SLIDE_NP2_RUN;
  //       end
  //     end
  //     SLIDE_NP2_RUN: begin
  //       // Reset the buffer-valid if the buffer is read, by default
  //       if (slide_np2_buf_valid_q && sldu_operand_ready_q[0])
  //         slide_np2_buf_valid_d = 1'b0;
  //       // Setup the current p2 stride
  //       sld_slamt = p2_stride_gen_stride_q;
  //       // Slide the operands as soon as valid
  //       if (&sldu_operand_valid) begin
  //         for (int unsigned l = 0; l < NrLanes; l++)
  //           result_queue_d[result_queue_write_pnt_q][l].wdata = sld_op_dst[l];
  //         slide_np2_buf_valid_d = 1'b1;
  //         // Operands correctly read
  //         sldu_operand_ready     = '1;
  //         // Update the p2 stride
  //         p2_stride_gen_update_d = 1'b1;
  //         // Commit the final result
  //         if (p2_stride_gen_popc_q == {'0, 1'b1} && result_queue_empty) begin
  //           state_d = SLIDE_NP2_COMMIT;
  //           // Prepare the write pointer
  //           result_queue_write_pnt_d = NP2_RESULT_PNT;
  //         end
  //       end
  //     end
  //     SLIDE_NP2_WAIT: begin
  //       if (result_queue_empty) begin
  //         result_queue_read_pnt_d  = NP2_RESULT_PNT;
  //         result_queue_write_pnt_d = NP2_RESULT_PNT;
  //         state_d = SLIDE_NP2_COMMIT;
  //       end
  //     end
  //     default:;
  //   endcase

    //////////////////////////////////
    //  Write results into the VRF  //
    //////////////////////////////////

    sldu_result_req_o   = result_queue_valid_q[result_queue_read_pnt_q] & (~(vinsn_commit.vfu inside {VFU_Alu, VFU_MFpu}));
    sldu_red_valid_o    = result_queue_valid_q[result_queue_read_pnt_q] & (vinsn_commit.vfu inside {VFU_Alu, VFU_MFpu});
    sldu_result_addr_o  = result_queue_q[result_queue_read_pnt_q].addr;
    sldu_result_id_o    = result_queue_q[result_queue_read_pnt_q].id;
    sldu_result_wdata_o = result_queue_q[result_queue_read_pnt_q].wdata;
    sldu_result_be_o    = result_queue_q[result_queue_read_pnt_q].be;

    // Update the final gnt vector
    result_final_gnt_d |= sldu_result_final_gnt_i;

    // Received a grant from the VRF (slide) or from the FUs (reduction).
    // Deactivate the request, but do not bump the pointers for now.
    if (((vinsn_commit.vfu inside {VFU_Alu, VFU_MFpu} && sldu_red_valid_o) || sldu_result_req_o) && sldu_result_gnt_i) begin
      result_queue_valid_d[result_queue_read_pnt_q] = 1'b0;
      result_queue_d[result_queue_read_pnt_q]       = '0;
      // Reset the final gnt vector since we are now waiting for another final gnt
      result_final_gnt_d = 1'b0;
    end
    
    // All lanes accepted the VRF request
    // If this was the last request, wait for all the final grants!
    // If this is a reduction, no need for the final grants
    if (!(|result_queue_valid_d[result_queue_read_pnt_q]) &&
      (vinsn_commit.vfu inside {VFU_Alu, VFU_MFpu} || (&result_final_gnt_d || commit_cnt_q > 8)))
      // There is something waiting to be written
      if (!result_queue_empty) begin
        // if (state_q != SLIDE_NP2_SETUP)
          // Increment the read pointer
          if (result_queue_read_pnt_q == ResultQueueDepth-1)
            result_queue_read_pnt_d = 0;
          else
            result_queue_read_pnt_d = result_queue_read_pnt_q + 1;

        // Decrement the counter of results waiting to be written
        result_queue_cnt_d -= 1;

        // Decrement the counter of remaining vector elements waiting to be written
        commit_cnt_d = commit_cnt_q - 8;
        if (commit_cnt_q < 8)
          commit_cnt_d = '0;
      end

    // Finished committing the results of a vector instruction
    if (vinsn_commit_valid && commit_cnt_d == '0) begin
      // Mark the vector instruction as being done
      pe_resp.vinsn_done[vinsn_commit.id] = 1'b1;

      // Update the commit counters and pointers
      vinsn_queue_d.commit_cnt -= 1;
      if (vinsn_queue_d.commit_pnt == VInsnQueueDepth-1)
        vinsn_queue_d.commit_pnt = '0;
      else
        vinsn_queue_d.commit_pnt += 1;

      // Update the commit counter for the next instruction
      if (vinsn_queue_d.commit_cnt != '0) begin
        commit_cnt_d = vinsn_queue_q.vinsn[vinsn_queue_d.commit_pnt].op inside {VSLIDEUP, VSLIDEDOWN}
                     ? vinsn_queue_q.vinsn[vinsn_queue_d.commit_pnt].vl << int'(vinsn_queue_q.vinsn[vinsn_queue_d.commit_pnt].vtype.vsew)
                     : (NrLanes * ($clog2(NrLanes) + 1)) << EW64;

        // Trim vector elements which are not written by the slide unit
        if (vinsn_queue_q.vinsn[vinsn_queue_d.commit_pnt].op == VSLIDEUP)
          commit_cnt_d -= vinsn_queue_q.vinsn[vinsn_queue_d.commit_pnt].stride[$bits(commit_cnt_d)-1:0];
      end
    end

    //////////////////////////////
    //  Accept new instruction  //
    //////////////////////////////

    if (!vinsn_queue_full && pe_req_valid_i && !vinsn_running_q[pe_req_i.id] &&
      (pe_req_i.vfu == VFU_SlideUnit || pe_req_i.op inside {[VREDSUM:VWREDSUM], [VFREDUSUM:VFWREDOSUM]})) begin
      vinsn_queue_d.vinsn[vinsn_queue_q.accept_pnt] = pe_req_i;
      vinsn_running_d[pe_req_i.id]                  = 1'b1;

      // Calculate the slide offset inside the vector register
      if (pe_req_i.op inside {VSLIDEUP, VSLIDEDOWN})
        vinsn_queue_d.vinsn[vinsn_queue_q.accept_pnt].stride = pe_req_i.stride <<
          int'(pe_req_i.vtype.vsew);
      // Always move 64-bit packs of data from one lane to the other
      if (pe_req_i.vfu inside {VFU_Alu, VFU_MFpu})
        vinsn_queue_d.vinsn[vinsn_queue_q.accept_pnt].vtype.vsew = EW64;

      if (vinsn_queue_d.commit_cnt == '0) begin
        // commit_cnt_d = pe_req_i.op inside {VSLIDEUP, VSLIDEDOWN}
        //              ? pe_req_i.vl << (int'(pe_req_i.vtype.vsew) - $clog2(NrLanes))
        //              : (NrLanes * ($clog2(NrLanes) + 1)) << EW64;
        commit_cnt_d = pe_req_i.vl << int'(pe_req_i.vtype.vsew);
        commit_cnt_d = commit_cnt_d >> $clog2(NrLanes);
        // Trim vector elements which are not written by the slide unit
        // VSLIDE1UP always writes at least 1 element
        if (pe_req_i.op == VSLIDEUP && !(pe_req_i.use_scalar_op && lane_id_i == 0)) begin
          commit_cnt_d -= vinsn_queue_d.vinsn[vinsn_queue_q.accept_pnt].stride[$bits(commit_cnt_d)-1:0];
        end
      end

      // Bump pointers and counters of the vector instruction queue
      vinsn_queue_d.accept_pnt += 1;
      vinsn_queue_d.issue_cnt += 1;
      vinsn_queue_d.commit_cnt += 1;
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      vinsn_running_q       <= '0;
      issue_cnt_q           <= '0;
      mask_cnt_q            <= '0;
      commit_cnt_q          <= '0;
      in_pnt_q              <= '0;
      out_pnt_q             <= '0;
      vrf_pnt_q             <= '0;
      output_limit_q        <= '0;
      slide_state_q         <= SLIDE_IDLE;
      sldu_vinsn_done_o     <= '0;
      result_final_gnt_q    <= '0;
      // red_stride_cnt_q      <= 1;
      slide_cnt_q           <= '0;
      first_slide_q         <= '0;
      first_mask_q          <= '0;
      amt_q                 <= '0;
      dir_q                 <= '0;
      sldu_valid_q          <= '0;
      sldu_ready_q          <= '0;
      slide_fin_valid_next_cycle <= '0;
    end else begin
      vinsn_running_q       <= vinsn_running_d;
      issue_cnt_q           <= issue_cnt_d;
      mask_cnt_q            <= mask_cnt_d;
      commit_cnt_q          <= commit_cnt_d;
      in_pnt_q              <= in_pnt_d;
      out_pnt_q             <= out_pnt_d;
      vrf_pnt_q             <= vrf_pnt_d;
      output_limit_q        <= output_limit_d;
      slide_state_q         <= slide_state_d;
      sldu_vinsn_done_o     <= pe_resp;
      result_final_gnt_q    <= result_final_gnt_d;
      // red_stride_cnt_q      <= red_stride_cnt_d;
      slide_cnt_q           <= slide_cnt_d;
      first_slide_q         <= first_slide_d;
      first_mask_q          <= first_mask_d;
      amt_q                 <= amt_d;
      dir_q                 <= dir_d;
      sldu_valid_q          <= sldu_valid_d;
      sldu_ready_q          <= sldu_ready_d;
      slide_fin_valid_next_cycle <= slide_fin_valid;
    end
  end

endmodule: sldu