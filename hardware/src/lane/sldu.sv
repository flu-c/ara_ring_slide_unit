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
    input  logic                              sldu_sync_i,
    output logic                              sldu_sync_start_o,
    output logic                              sldu_sync_fin_o,
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

  localparam int unsigned ResultQueueDepth = 2;

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

  //////////////////////////
  //  Cut from the masku  //
  //////////////////////////

  logic  mask_ready_d;
  logic  mask_ready_q;
  logic mask_valid;

  // Sample only SLDU mask valid
  assign mask_valid = mask_valid_i & ~vinsn_issue_q.vm & vinsn_issue_valid_q;
  

  // Don't upset the masku with a spurious ready
  assign mask_ready_o = mask_ready_d & mask_valid_i & ~vinsn_issue_q.vm & vinsn_issue_valid_q & !(vinsn_issue_q.vfu inside {VFU_Alu, VFU_MFpu});

  typedef logic [idx_width(8*NrLanes)-1:0] stride_t;

  //////////////////
  //  Reductions  //  // TODO //
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

  // Shouldn't be necessary to change
  localparam int unsigned SlideQueueDepth = 2;

  // Slide queue
  elen_t slide_queue_in, slide_queue_out;
  logic  slide_queue_push, slide_queue_pop, slide_queue_empty, slide_queue_full;

  // FIFO to break handshake chain
  fifo_v3 #(
    .DATA_WIDTH (ELEN             ),
    .DEPTH      (SlideQueueDepth  )
  ) i_slide_queue (
    .clk_i      (clk_i            ),
    .rst_ni     (rst_ni           ),
    .flush_i    ('0               ),
    .testmode_i ('0               ),
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
  elen_t shuffle_out;
  elen_t slide_result;
  elen_t write_back;

  // Input and output eew for reshuffling
  rvv_pkg::vew_e sld_eew_src;
  rvv_pkg::vew_e sld_eew_dst;

  // 0: slidedown, 1: slideup
  logic sld_dir;

  // How many slides are necessray
  stride_t sld_slamt;
  // How much shuffling is necessary
  logic   [idx_width(StrbWidth)-1:0] shuffle_amt_d, shuffle_amt_q;

  // Max Slide: NrLanes/2, Min Slide: 0
  logic [idx_width(NrLanes)-1:0] amt_d, amt_q;

  // slide_cnt counts how many slides are already done
  // can count up or down depending on the slide direction
  logic [idx_width(NrLanes)-1:0] slide_cnt_d, slide_cnt_q;

  sldu_op_dp #(
  ) i_sldu_op_dp (
    .op_i     (slide_fin  ),
    .slamt_i  (shuffle_amt_q),
    .eew_src_i(sld_eew_src),
    .eew_dst_i(sld_eew_dst),
    .op_o     (shuffle_out)
  );

  ///////////////////////
  //  Write back FIFO  //
  ///////////////////////

  logic  write_back_push, write_back_pop, write_back_empty, write_back_full;

  // Makes the write back independent from the slide FSM
  fifo_v3 #(
    .DATA_WIDTH (ELEN            ),
    .DEPTH      (2               )
  ) i_write_back (
    .clk_i      (clk_i           ),
    .rst_ni     (rst_ni          ),
    .flush_i    ('0              ),
    .testmode_i ('0              ),
    .full_o     (write_back_full ),
    .empty_o    (write_back_empty),
    .usage_o    (/* Unused */    ),
    .data_i     (slide_result    ),
    .push_i     (write_back_push ),
    .data_o     (write_back      ),
    .pop_i      (write_back_pop  )
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

  // How many elements are left to slide
  vlen_t slides_left_d, slides_left_q;

  // First slide important for write back
  logic  first_slide_d, first_slide_q;

  // vslide1down needs an extra element at the end, control signal
  logic  vslide1down_d, vslide1down_q;

  // Every sldu always writes back twice to keep them in sync
  logic wb_stall_d, wb_stall_q;
  logic wb_first, wb_second;

  // Remaining bytes of the current instruction in the commit phase
  vlen_t commit_cnt_d, commit_cnt_q;

  logic  [7:0] out_en_flat, out_en_seq;
  strb_t       out_en;
  
  // Pointers in the input operand and the output result
  logic   [idx_width(StrbWidth):0] in_pnt_d, in_pnt_q;
  logic   [idx_width(StrbWidth):0] out_pnt_d, out_pnt_q;
  vaddr_t                          vrf_pnt_d, vrf_pnt_q;

  // Respected by default: input_limit_d  = 8*NrLanes + out_pnt_d - in_pnt_d;
  // To enforce: output_limit_d = out_pnt_d + issue_cnt_d;
  logic [idx_width(MAXVL+1):0] output_limit_d, output_limit_q;

  // handshakes with other sldu's
  logic sldu_valid_prev_d, sldu_valid_next_d, sldu_valid_prev_q, sldu_valid_next_q;
  logic sldu_ready_d, sldu_ready_q;

  // incoming new operand
  logic slide_op_valid;
  elen_t slide_op;

  always_comb begin
    // Maintain state
    slide_state_d          = slide_state_q;
    slide_cnt_d            = slide_cnt_q;
    issue_cnt_d            = issue_cnt_q;
    first_slide_d          = first_slide_q;
    amt_d                  = amt_q;
    commit_cnt_d           = commit_cnt_q;
    vinsn_queue_d          = vinsn_queue_q;
    result_final_gnt_d     = result_final_gnt_q;
    sldu_valid_prev_d      = sldu_valid_prev_q;
    sldu_valid_next_d      = sldu_valid_next_q;
    sldu_ready_d           = sldu_ready_q;
    vrf_pnt_d              = vrf_pnt_q;
    output_limit_d         = output_limit_q;
    slides_left_d          = slides_left_q;
    shuffle_amt_d          = shuffle_amt_q;
    vslide1down_d          = vslide1down_q;
    in_pnt_d               = in_pnt_q;
    out_pnt_d              = out_pnt_q;

    result_queue_d           = result_queue_q;
    result_queue_valid_d     = result_queue_valid_q;
    result_queue_read_pnt_d  = result_queue_read_pnt_q;
    result_queue_write_pnt_d = result_queue_write_pnt_q;
    result_queue_cnt_d       = result_queue_cnt_q;

    slide_queue_push       = 1'b0;
    slide_queue_pop        = 1'b0;
    sldu_operand_ready_o   = 1'b0;
    mask_ready_d           = 1'b0;
    write_back_push        = 1'b0;
    write_back_pop         = 1'b0;
    wb_stall_d             = 1'b0;
    wb_first               = 1'b0;
    sldu_sync_start_o      = 1'b0;
    sldu_sync_fin_o        = 1'b0;
    pe_resp                = '0;
    out_en_flat            = '0;
    out_en_seq             = '0;
    out_en                 = '0;
    slide_fin              = '0;
    slide_queue_in         = '0;

    // Vector instructions currently running
    vinsn_running_d = vinsn_running_q & pe_vinsn_running_i;

    sld_eew_src = (vinsn_issue_q.vfu inside {VFU_Alu, VFU_MFpu})
                ? vinsn_issue_q.vtype.vsew
                : vinsn_issue_q.eew_vs2;
    sld_eew_dst = vinsn_issue_q.vtype.vsew;
    sld_dir     = (vinsn_issue_q.op == VSLIDEUP) || (vinsn_issue_q.vfu inside {VFU_Alu, VFU_MFpu});
    sld_slamt   = stride_t'(vinsn_issue_q.stride >> vinsn_issue_q.vtype.vsew);

    // New operand from lane op queue
    slide_op_valid = sldu_operand_target_fu_i == ALU_SLDU ? sldu_operand_valid_i : 1'b0;
    slide_op       = sldu_operand_i;

    // Outputs
    sldu_prev_valid_o = sldu_valid_prev_q && ~write_back_full;
    sldu_next_valid_o = sldu_valid_next_q && ~write_back_full;
    sldu_prev_ready_o = sldu_ready_q && ~write_back_full;
    sldu_next_ready_o = sldu_ready_q && ~write_back_full;
    sldu_prev_data_o  = slide_queue_out;
    sldu_next_data_o  = slide_queue_out;

    /////////////////
    //  Slide FSM  //
    /////////////////

    unique case (slide_state_q)
      SLIDE_IDLE: begin
        // start new slide
        if (slide_op_valid && vinsn_issue_valid_q && ~sldu_sync_i) begin

          vrf_pnt_d = '0;

          // All sldu's need to finish a instruction before any starts a new one
          sldu_sync_start_o = 1'b1;

          unique case (vinsn_issue_q.op)
            VSLIDEUP: begin
              automatic int slamt = sld_slamt - 1;
              amt_d = NrLanes - sld_slamt[idx_width(NrLanes)-1:0];
              // vslideup starts reading the source operand from its beginning
              in_pnt_d  = '0;

              // vslideup starts writing the destination vector at the slide offset
              out_pnt_d = slamt[idx_width(NrLanes)+3:idx_width(NrLanes)];
              if (lane_id_i <= slamt[idx_width(NrLanes)-1:0])
                out_pnt_d += 1;
              shuffle_amt_d = out_pnt_d;
              out_pnt_d = out_pnt_d << vinsn_issue_q.vtype.vsew;

              // Start writing at the middle of the destination vector
              vrf_pnt_d = vinsn_issue_q.stride >> $clog2(8*NrLanes);

              // Initialize counters
              issue_cnt_d = vinsn_issue_q.vl << int'(vinsn_issue_q.vtype.vsew);
              issue_cnt_d = issue_cnt_d >> $clog2(NrLanes);
              output_limit_d = issue_cnt_d;
              if (vrf_pnt_d != 0) begin
                issue_cnt_d -= vrf_pnt_d << 3;
                commit_cnt_d -= vrf_pnt_d << 3;
                output_limit_d -= vrf_pnt_d << 3;
                if (out_pnt_d != 8) begin
                  vrf_pnt_d += 1;
                  commit_cnt_d -= 8;
                  output_limit_d -= 8;
                end else begin
                  out_pnt_d = 8;
                  vrf_pnt_d -= 1;
                  commit_cnt_d += 8;
                  output_limit_d += 8;
                end
              end

              // Initialize be-enable-generation ancillary signals
              slides_left_d  = issue_cnt_d;

              // Only needed for vslide1up to write the first element
              first_slide_d = vinsn_issue_q.use_scalar_op && lane_id_i == 0 ? 1'b1 : 1'b0;
            end
            VSLIDEDOWN: begin
              automatic int last_line = vinsn_issue_q.vl[idx_width(NrLanes)-1:0] - 1;
              automatic int slamt = sld_slamt;
              automatic int extra_stride;
              automatic int tot_vl;
              amt_d = sld_slamt[idx_width(NrLanes)-1:0];
              // vslidedown starts reading the source operand from the slide offset
              if (vinsn_issue_q.vtype.vsew != EW64) begin
                in_pnt_d = int'(slamt[idx_width(NrLanes)+3:idx_width(NrLanes)] << vinsn_issue_q.vtype.vsew);
                // not all lanes start from the same offset
                if (sld_slamt[idx_width(NrLanes)-1:0] >= NrLanes - lane_id_i)
                  in_pnt_d += int'(1 << vinsn_issue_q.vtype.vsew);
              // special case for EW64
              end else begin
                  in_pnt_d = (sld_slamt[idx_width(NrLanes)-1:0] >= NrLanes - lane_id_i) ? 8 : 0;
              end
              shuffle_amt_d = int'((8 - in_pnt_d) >> vinsn_issue_q.vtype.vsew);

              // vslidedown starts writing the destination vector at its beginning
              out_pnt_d = '0;

              // The stride move the initial address in boundaries of 8*NrLanes Byte.
              // If the stride is not multiple of a full VRF word (8*NrLanes Byte),
              // we must request it as well from the VRF

              // Find the number of extra elements to ask, related to the stride
              unique case (vinsn_issue_q.vtype.vsew)
                EW8 : extra_stride = slamt[$clog2(8*NrLanes)-1:0];
                EW16: extra_stride = {1'b0, slamt[$clog2(4*NrLanes)-1:0]};
                EW32: extra_stride = {2'b0, slamt[$clog2(2*NrLanes)-1:0]};
                EW64: extra_stride = {3'b0, slamt[$clog2(1*NrLanes)-1:0]};
                default:
                  extra_stride = {3'b0, slamt[$clog2(1*NrLanes)-1:0]};
              endcase

              // Initialize counters

              // Find the total number of elements to be asked
              tot_vl = vinsn_issue_q.vl << int'(vinsn_issue_q.vtype.vsew);
              if (!vinsn_issue_q.use_scalar_op)
                tot_vl += extra_stride;
              slides_left_d = tot_vl >> $clog2(NrLanes);
              if (slides_left_d << $clog2(NrLanes) != tot_vl)
                slides_left_d += 1 << int'(vinsn_issue_q.vtype.vsew);

              issue_cnt_d = vinsn_issue_q.vl << int'(vinsn_issue_q.vtype.vsew);
              issue_cnt_d = issue_cnt_d >> $clog2(NrLanes);

              if (vinsn_issue_q.use_scalar_op && (last_line[idx_width(NrLanes)-1:0] == lane_id_i))
                issue_cnt_d -= 1 << int'(vinsn_issue_q.vtype.vsew);

              if (vinsn_issue_q.use_scalar_op && (last_line[idx_width(NrLanes)-1:0] == lane_id_i) && (vinsn_issue_q.vtype.vsew == EW64)) begin //
                vslide1down_d = 1'b1;
                issue_cnt_d += 8;
              end
              output_limit_d = issue_cnt_d;

              first_slide_d = 1'b1;
            end
          endcase

          // Determine next state
          if (amt_d == 0) begin
            slide_state_d = SLIDE_NO_SLIDE;
          end else begin
            slide_state_d = (amt_d == 1 || (NrLanes - amt_d) == 1) ? SLIDE_LAST : SLIDE_RUN;
            // Load the first operand into the slide queue
            sldu_operand_ready_o = 1'b1;
            slide_cnt_d = '0;
            slide_queue_in = slide_op;
            slide_queue_push = 1'b1;
            sldu_ready_d = 1'b1;
            // Determines slide direction
            if (amt_d[idx_width(NrLanes)-1])
              sldu_valid_next_d = 1'b1;
            if (~amt_d[idx_width(NrLanes)-1])
              sldu_valid_prev_d = 1'b1;
          end
        end
      end
      // No slide needed, directly goes to WB until no operands left
      SLIDE_NO_SLIDE: begin
        if (~write_back_full && slide_op_valid && vinsn_issue_valid_q) begin
          slide_fin = slide_op;
          write_back_push = 1'b1;
          sldu_operand_ready_o = 1'b1;
          slides_left_d = slides_left_q < 8 ? 0 : slides_left_q - 8;
        end else if (slides_left_q == 0) begin
            slide_state_d = SLIDE_IDLE;
        end
      end
      // Gets operand from neighboring sldu and delete slided operand until only one slide left
      SLIDE_RUN: begin
        // Slide to next lane
        if (amt_q[idx_width(NrLanes)-1]) begin
          // Get the operand from the prev lane
          if (sldu_prev_valid_i && sldu_ready_q && sldu_next_ready_i && sldu_valid_next_q) begin
            slide_queue_in = sldu_prev_data_i;
            slide_cnt_d -= 1;
            slide_queue_push = 1'b1;
            slide_queue_pop = 1'b1;
            if (slide_cnt_d - 1 == amt_q) begin
              slide_state_d = SLIDE_LAST;
            end
          end
        // Slide to prev lane
        end else begin
          // Get the operand from the next lane
          if(sldu_next_valid_i && sldu_ready_q && sldu_prev_ready_i && sldu_valid_prev_q) begin
            slide_queue_in = sldu_next_data_i;
            slide_cnt_d += 1;
            slide_queue_push = 1'b1;
            slide_queue_pop = 1'b1;
            if (slide_cnt_q + 2 == amt_q) begin
              slide_state_d = SLIDE_LAST;
            end
          end
        end
      end
      // Get operand from neighboring lane and direct it to WB, if possible load next operand
      SLIDE_LAST: begin
        if (~write_back_full && (slides_left_q != 0 || ~slide_queue_empty)) begin
          if (amt_q[idx_width(NrLanes)-1] && sldu_prev_valid_i && sldu_ready_q) begin
            // Slide from prev lane and finish
            slide_fin = sldu_prev_data_i;
            write_back_push = 1'b1;
            sldu_ready_d = 1'b0;
            slides_left_d = slides_left_q < 8 ? 0 : slides_left_q - 8;
          end else if (~amt_q[idx_width(NrLanes)-1] && sldu_next_valid_i && sldu_ready_q) begin
            // Slide from next lane and finish
            slide_fin = sldu_next_data_i;
            write_back_push = 1'b1;
            sldu_ready_d = 1'b0;
            slides_left_d = slides_left_q < 8 ? 0 : slides_left_q - 8;
          end
          // Delete operand in FIFO
          if (amt_q[idx_width(NrLanes)-1] && sldu_valid_next_q && sldu_next_ready_i) begin
            sldu_valid_next_d = 1'b0;
            slide_queue_pop = 1'b1;
          end else if (~amt_q[idx_width(NrLanes)-1] && sldu_valid_prev_q && sldu_prev_ready_i) begin
            sldu_valid_prev_d = 1'b0;
            slide_queue_pop = 1'b1;
          end
          // Start a new slide
          if (slide_op_valid && vinsn_issue_valid_q && slide_queue_empty && slides_left_d > 0 && ~sldu_ready_q) begin
            sldu_operand_ready_o = 1'b1;
            slide_cnt_d = '0;
            slide_queue_in = slide_op;
            slide_queue_push = 1'b1;
            sldu_ready_d = 1'b1;
            // Determine slide direction
            if (amt_q[idx_width(NrLanes)-1])
              sldu_valid_next_d = 1'b1;
            if (~amt_q[idx_width(NrLanes)-1])
              sldu_valid_prev_d = 1'b1;
            // Determine next state
            slide_state_d = (amt_q == 1 || (NrLanes - amt_q) == 1) ? SLIDE_LAST : SLIDE_RUN;
          end
        // Extra 'empty' element is put in WB queue after everyting is slided, which is replaced by the scalar 
        // of the vslide1down instruction
        end else if (slides_left_q == 0 && ~vslide1down_q && ~write_back_full) begin
          slide_state_d = SLIDE_IDLE;
        end else if (slides_left_q == 0 && vslide1down_q && ~write_back_full) begin
          write_back_push = 1'b1;
          vslide1down_d = 1'b0;
        end
      end
    endcase

    // Determines which lanes actually need to shuffle the operand
    if (sld_dir) begin
      slide_result = lane_id_i < sld_slamt ? shuffle_out : slide_fin;
    end else begin
      slide_result = NrLanes-1 - lane_id_i < sld_slamt ? shuffle_out : slide_fin;
    end

    /////////////////////////////
    //  Write to result queue  //
    /////////////////////////////

    // pop element if in the waiting cycle
    if (wb_stall_q)
      write_back_pop = 1'b1;

    if (~result_queue_full && ~write_back_empty && (vinsn_issue_q.vm || mask_valid) && ~wb_stall_q) begin
      // How many bytes are we copying from the operand to the destination, in this cycle?
      automatic int in_byte_count = 8 - in_pnt_q;
      automatic int out_byte_count = 8 - out_pnt_q;
      automatic int byte_count = in_byte_count < out_byte_count ? in_byte_count : out_byte_count;
      automatic int first_slide_byte_count = (first_slide_q && vinsn_issue_q.op == VSLIDEUP) ? byte_count + (1 << int'(vinsn_issue_q.vtype.vsew)) : byte_count;

      // Always take two cycles for WB to keep sldu's in sync
      if (byte_count == 8 && ~wb_second && ~(issue_cnt_q <= 16))
        wb_stall_d = 1'b1;
      if (~wb_second)
        wb_first = 1'b1;

      first_slide_d = 1'b0;

      if (in_pnt_q == 0)
        output_limit_d = output_limit_q < 8 ? '0 : output_limit_q - 8;

      // The first VSLIDE(1)DOWN can discard the first elements of the first slide and only needs one cycle for WB
      if (first_slide_q && vinsn_issue_q.op == VSLIDEDOWN) begin
        write_back_pop = 1'b1;
        wb_stall_d = 1'b0;
        wb_first = 1'b0;
      end

      // Build the sequential byte-output-enable
      for (int unsigned b = 0; b < 8; b++)
        if ((b >= out_pnt_q && b < output_limit_q) || vinsn_issue_q.vfu inside {VFU_Alu, VFU_MFpu})
          out_en_seq[b] = 1'b1;

      // Shuffle the output enable
      for (int unsigned b = 0; b < 8; b++)
        out_en_flat[shuffle_index(b, 1, vinsn_issue_q.vtype.vsew)] = out_en_seq[b];

      // Mask the output enable with the mask vector
      out_en = out_en_flat & ({8*NrLanes{vinsn_issue_q.vm | (vinsn_issue_q.vfu inside {VFU_Alu, VFU_MFpu})}} | mask_i);

      // Write in the correct bytes
      for (int b = 0; b < 8; b++)
        if (out_en[b]) begin
          result_queue_d[result_queue_write_pnt_q].wdata[8*b +: 8] = write_back[8*b +: 8];
          result_queue_d[result_queue_write_pnt_q].be[b]           = 1'b1;
        end

      // Initialize id and addr fields of the result queue requests
      result_queue_d[result_queue_write_pnt_q].id   = vinsn_issue_q.id;
      result_queue_d[result_queue_write_pnt_q].addr = vaddr(vinsn_issue_q.vd, NrLanes) + vrf_pnt_q;

      // Bump pointers (reductions always finish in one shot)
      in_pnt_d    = vinsn_issue_q.vfu inside {VFU_Alu, VFU_MFpu} ? 8               : in_pnt_q  + byte_count;
      out_pnt_d   = vinsn_issue_q.vfu inside {VFU_Alu, VFU_MFpu} ? 8               : out_pnt_q + byte_count;
      issue_cnt_d = vinsn_issue_q.vfu inside {VFU_Alu, VFU_MFpu} ? issue_cnt_q - 8 : issue_cnt_q - first_slide_byte_count;

      // Read a full word from the VRF or finished the instruction
      if (in_pnt_d == 8 || issue_cnt_q <= first_slide_byte_count) begin
        // Reset the pointer
        in_pnt_d = '0;
        if (~wb_stall_d)
          write_back_pop = 1'b1;
      end

      // Filled up a word to the VRF or finished the instruction
      if ((out_pnt_d == 8 || issue_cnt_q <= byte_count) && output_limit_q != 0) begin //
        // Reset the pointer
        out_pnt_d = '0;

        // We used all the bits of the mask
        if (vinsn_issue_q.op inside {VSLIDEUP, VSLIDEDOWN})
          mask_ready_d = !vinsn_issue_q.vm;

        // Increment VRF address
        vrf_pnt_d = vrf_pnt_q + 1;

        // Send result to the VRF
        result_queue_cnt_d += 1;
        result_queue_valid_d[result_queue_write_pnt_q] = '1;
        result_queue_write_pnt_d                       = result_queue_write_pnt_q + 1;
        if (result_queue_write_pnt_q == ResultQueueDepth-1)
          result_queue_write_pnt_d = '0;
      end

      // Finished the operation
      if (issue_cnt_q <= first_slide_byte_count || (vinsn_issue_q.vfu inside {VFU_Alu, VFU_MFpu} && issue_cnt_q <= 8)) begin
        automatic int last_line = vinsn_issue_q.vl[idx_width(NrLanes)-1:0] - 1;
        // If this is a vslide1down, fill up the last position with the scalar operand
        if (vinsn_issue_q.op == VSLIDEDOWN && vinsn_issue_q.use_scalar_op && (last_line[idx_width(NrLanes)-1:0] == lane_id_i)) begin
          // Copy the scalar operand to the last word
          automatic int out_seq_byte = output_limit_q;
          automatic int out_byte = shuffle_index(out_seq_byte, 1, vinsn_issue_q.vtype.vsew);

          unique case (vinsn_issue_q.vtype.vsew)
            EW8: begin
              result_queue_d[result_queue_write_pnt_q].wdata[8*out_byte +: 8]
                = vinsn_issue_q.scalar_op[7:0];
              result_queue_d[result_queue_write_pnt_q].be[out_byte +: 1] =
                vinsn_issue_q.vm || mask_i[out_byte];
            end
            EW16: begin
              result_queue_d[result_queue_write_pnt_q].wdata[8*out_byte +: 16]
                = vinsn_issue_q.scalar_op[15:0];
              result_queue_d[result_queue_write_pnt_q].be[out_byte +: 2] =
                {2{vinsn_issue_q.vm || mask_i[out_byte]}};
            end
            EW32: begin
              result_queue_d[result_queue_write_pnt_q].wdata[8*out_byte +: 32]
                = vinsn_issue_q.scalar_op[31:0];
              result_queue_d[result_queue_write_pnt_q].be[out_byte +: 4] =
                {4{vinsn_issue_q.vm || mask_i[out_byte]}};
            end
            EW64: begin
              result_queue_d[result_queue_write_pnt_q].wdata[8*out_byte +: 64]
                = vinsn_issue_q.scalar_op[63:0];
              result_queue_d[result_queue_write_pnt_q].be[out_byte +: 8] =
                {8{vinsn_issue_q.vm || mask_i[out_byte]}};
            end
          endcase
        end
        // Reset the logarighmic counter
        // // red_stride_cnt_d = 1;

        // Only here in order to increase the issue_pnt only once
        issue_cnt_d = 16;

        // Increment vector instruction queue pointers and counters
        vinsn_queue_d.issue_pnt += 1;
        vinsn_queue_d.issue_cnt -= 1;
      end

      // If this is a vslide1up instruction, copy the scalar operand to the first word
      if (first_slide_q && vinsn_issue_q.op == VSLIDEUP) begin
        // issue_cnt_d -= 1 << int'(vinsn_issue_q.vtype.vsew);
        unique case (vinsn_issue_q.vtype.vsew)
          EW8: begin
            result_queue_d[result_queue_write_pnt_q].wdata[7:0] =
              vinsn_issue_q.scalar_op[7:0];
            result_queue_d[result_queue_write_pnt_q].be[0:0] =
              vinsn_issue_q.vm || mask_i[0];
          end
          EW16: begin
            result_queue_d[result_queue_write_pnt_q].wdata[15:0] =
              vinsn_issue_q.scalar_op[15:0];
            result_queue_d[result_queue_write_pnt_q].be[1:0] =
              {2{vinsn_issue_q.vm || mask_i[0]}};
          end
          EW32: begin
            result_queue_d[result_queue_write_pnt_q].wdata[31:0] =
              vinsn_issue_q.scalar_op[31:0];
            result_queue_d[result_queue_write_pnt_q].be[3:0] =
              {4{vinsn_issue_q.vm || mask_i[0]}};
          end
          EW64: begin
            result_queue_d[result_queue_write_pnt_q].wdata[63:0] =
              vinsn_issue_q.scalar_op[63:0];
            result_queue_d[result_queue_write_pnt_q].be[7:0] =
              {8{vinsn_issue_q.vm || mask_i[0]}};
          end
        endcase
      end
    end


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
        if (commit_cnt_q <= 8) begin
          commit_cnt_d = '0;
        end
      end

    // Finished committing the results of a vector instruction
    if (vinsn_commit_valid && commit_cnt_d == '0) begin
      // Mark the vector instruction as being done
      pe_resp.vinsn_done[vinsn_commit.id] = 1'b1;
      // sldu_sync_d = 1'b0;
      sldu_sync_fin_o = 1'b1;

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
        commit_cnt_d = commit_cnt_d >> $clog2(NrLanes);

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
        commit_cnt_d = pe_req_i.vl << int'(pe_req_i.vtype.vsew);
        commit_cnt_d = commit_cnt_d >> $clog2(NrLanes);
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
      amt_q                 <= '0;
      sldu_valid_prev_q     <= '0;
      sldu_valid_next_q     <= '0;
      sldu_ready_q          <= '0;
      wb_stall_q            <= '0;
      wb_second             <= '0;
      slides_left_q         <= '0;
      shuffle_amt_q         <= '0;
      vslide1down_q         <= '0;
    end else begin
      vinsn_running_q       <= vinsn_running_d;
      issue_cnt_q           <= issue_cnt_d;
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
      amt_q                 <= amt_d;
      sldu_valid_prev_q     <= sldu_valid_prev_d;
      sldu_valid_next_q     <= sldu_valid_next_d;
      sldu_ready_q          <= sldu_ready_d;
      wb_stall_q            <= wb_stall_d;
      wb_second             <= wb_first;
      slides_left_q         <= slides_left_d;
      shuffle_amt_q         <= shuffle_amt_d;
      vslide1down_q         <= vslide1down_d;
    end
  end

endmodule: sldu