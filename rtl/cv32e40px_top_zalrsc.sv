// Copyright 2024 Dolphin Design
// SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1
//
// Licensed under the Solderpad Hardware License v 2.1 (the "License");
// you may not use this file except in compliance with the License, or,
// at your option, the Apache License version 2.0.
// You may obtain a copy of the License at
//
// https://solderpad.org/licenses/SHL-2.1/
//
// Unless required by applicable law or agreed to in writing, any work
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/////////////////////////////////////////////////////////////////////////////
//                                                                         //
// Contributors: Pascal Gouedo, Dolphin Design <pascal.gouedo@dolphin.fr>  //
//                                                                         //
// Description:  Top level module of CV32E40P instantiating the Core and   //
//               an optional CVFPU with its clock gating cell.             //
//                                 +                                       //
//               obi_atop_resolver to test lr.w & sc.w                     //
//                                                                         //
/////////////////////////////////////////////////////////////////////////////

// --- TIP FOR QUICK USAGE ---
// when you launch the commands < make comp ... > and < make test ... >,
// the makefiles look for the file name "cv32e40px_top.sv".
// Instead of dealing with them, it's quicker and easier to just rename this file:
// - "cv32e40px_top.sv" if you want to run any test that concerns the Zalrsc extension
//   (which means you need to instanciate the obi_atop_resolver);
// - "cv32e40px_top_zalrsc.sv" if you are not interested in this top wrapper.

`include "obi/typedef.svh"

module cv32e40px_top
  import cv32e40px_core_v_xif_pkg::*;
#(
    parameter COREV_X_IF = 0,
    parameter COREV_PULP = 0, // PULP ISA Extension (incl. custom CSRs and hardware loop, excl. cv.elw)
    parameter COREV_CLUSTER = 0,  // PULP Cluster interface (incl. cv.elw)
    parameter FPU = 0,  // Floating Point Unit (interfaced via APU interface)
    parameter FPU_ADDMUL_LAT = 0,  // Floating-Point ADDition/MULtiplication computing lane pipeline registers number
    parameter FPU_OTHERS_LAT = 0,  // Floating-Point COMParison/CONVersion computing lanes pipeline registers number
    parameter ZFINX = 0,  // Float-in-General Purpose registers
    parameter NUM_MHPMCOUNTERS = 1,
    parameter A_EXTENSION = 1  // RV32A extension (AMO instructions)
) (
    // Clock and Reset
    input logic clk_i,
    input logic rst_ni,

    input logic pulp_clock_en_i,  // PULP clock enable (only used if COREV_CLUSTER = 1)
    input logic scan_cg_en_i,  // Enable all clock gates for testing

    // Core ID, Cluster ID, debug mode halt address and boot address are considered more or less static
    input logic [31:0] boot_addr_i,
    input logic [31:0] mtvec_addr_i,
    input logic [31:0] dm_halt_addr_i,
    input logic [31:0] hart_id_i,
    input logic [31:0] dm_exception_addr_i,

    // Instruction memory interface
    output logic        instr_req_o,
    input  logic        instr_gnt_i,
    input  logic        instr_rvalid_i,
    output logic [31:0] instr_addr_o,
    input  logic [31:0] instr_rdata_i,

    // Data memory interface
    output logic        data_req_o,
    input  logic        data_gnt_i,
    input  logic        data_rvalid_i,
    output logic        data_we_o,
    output logic [ 3:0] data_be_o,
    output logic [31:0] data_addr_o,
    output logic [31:0] data_wdata_o,
    input  logic [31:0] data_rdata_i,
    output logic [ 5:0] data_atop_o,

    // CORE-V-XIF
    // Compressed interface
    output logic x_compressed_valid_o,
    input logic x_compressed_ready_i,
    output x_compressed_req_t x_compressed_req_o,
    input x_compressed_resp_t x_compressed_resp_i,

    // Issue Interface
    output logic x_issue_valid_o,
    input logic x_issue_ready_i,
    output x_issue_req_t x_issue_req_o,
    input x_issue_resp_t x_issue_resp_i,

    // Commit Interface
    output logic x_commit_valid_o,
    output x_commit_t x_commit_o,

    // Memory request/response Interface
    input logic x_mem_valid_i,
    output logic x_mem_ready_o,
    input x_mem_req_t x_mem_req_i,
    output x_mem_resp_t x_mem_resp_o,

    // Memory Result Interface
    output logic x_mem_result_valid_o,
    output x_mem_result_t x_mem_result_o,

    // Result Interface
    input logic x_result_valid_i,
    output logic x_result_ready_o,
    input x_result_t x_result_i,

    // Interrupt inputs
    input  logic [31:0] irq_i,  // CLINT interrupts + CLINT extension interrupts
    output logic        irq_ack_o,
    output logic [ 4:0] irq_id_o,

    // Debug Interface
    input  logic debug_req_i,
    output logic debug_havereset_o,
    output logic debug_running_o,
    output logic debug_halted_o,

    // CPU Control Signals
    input  logic fetch_enable_i,
    output logic core_sleep_o
);

  import cv32e40px_apu_core_pkg::*;
  import obi_pkg::*;

  // Core to FPU
  logic                              apu_busy;
  logic                              apu_req;
  logic [   APU_NARGS_CPU-1:0][31:0] apu_operands;
  logic [     APU_WOP_CPU-1:0]       apu_op;
  logic [APU_NDSFLAGS_CPU-1:0]       apu_flags;

  // FPU to Core
  logic                              apu_gnt;
  logic                              apu_rvalid;
  logic [                31:0]       apu_rdata;
  logic [APU_NUSFLAGS_CPU-1:0]       apu_rflags;

  logic apu_clk_en, apu_clk;

  // connection between core & obi_atop_resolver
  logic        data_req_core;
  logic        data_gnt_core;
  logic        data_rvalid_core;
  logic        data_we_core;
  logic [ 3:0] data_be_core;
  logic [31:0] data_addr_core;
  logic [31:0] data_wdata_core;
  logic [31:0] data_rdata_core;
  logic [ 5:0] data_atop_core;

  // Instantiate the Core
  cv32e40px_core #(
      .COREV_X_IF      (COREV_X_IF),
      .COREV_PULP      (COREV_PULP),
      .COREV_CLUSTER   (COREV_CLUSTER),
      .FPU             (FPU),
      .FPU_ADDMUL_LAT  (FPU_ADDMUL_LAT),
      .FPU_OTHERS_LAT  (FPU_OTHERS_LAT),
      .ZFINX           (ZFINX),
      .NUM_MHPMCOUNTERS(NUM_MHPMCOUNTERS),
      .A_EXTENSION     (A_EXTENSION)
  ) core_i (
      .clk_i (clk_i),
      .rst_ni(rst_ni),

      .pulp_clock_en_i(pulp_clock_en_i),
      .scan_cg_en_i   (scan_cg_en_i),

      .boot_addr_i        (boot_addr_i),
      .mtvec_addr_i       (mtvec_addr_i),
      .dm_halt_addr_i     (dm_halt_addr_i),
      .hart_id_i          (hart_id_i),
      .dm_exception_addr_i(dm_exception_addr_i),

      .instr_req_o   (instr_req_o),
      .instr_gnt_i   (instr_gnt_i),
      .instr_rvalid_i(instr_rvalid_i),
      .instr_addr_o  (instr_addr_o),
      .instr_rdata_i (instr_rdata_i),

      // data memory interface connected to obi_atop_resolver
      .data_req_o   (data_req_core),
      .data_gnt_i   (data_gnt_core),
      .data_rvalid_i(data_rvalid_core),
      .data_we_o    (data_we_core),
      .data_be_o    (data_be_core),
      .data_addr_o  (data_addr_core),
      .data_wdata_o (data_wdata_core),
      .data_rdata_i (data_rdata_core),
      .data_atop_o  (data_atop_core),

      .apu_busy_o    (apu_busy),
      .apu_req_o     (apu_req),
      .apu_gnt_i     (apu_gnt),
      .apu_operands_o(apu_operands),
      .apu_op_o      (apu_op),
      .apu_flags_o   (apu_flags),
      .apu_rvalid_i  (apu_rvalid),
      .apu_result_i  (apu_rdata),
      .apu_flags_i   (apu_rflags),

      // CORE-V-XIF
      // Compressed interface
      .x_compressed_valid_o(x_compressed_valid_o),
      .x_compressed_ready_i(x_compressed_ready_i),
      .x_compressed_req_o  (x_compressed_req_o),
      .x_compressed_resp_i (x_compressed_resp_i),

      // Issue Interface
      .x_issue_valid_o(x_issue_valid_o),
      .x_issue_ready_i(x_issue_ready_i),
      .x_issue_req_o  (x_issue_req_o),
      .x_issue_resp_i (x_issue_resp_i),

      // Commit Interface
      .x_commit_valid_o(x_commit_valid_o),
      .x_commit_o(x_commit_o),

      // Memory request/response Interface
      .x_mem_valid_i(x_mem_valid_i),
      .x_mem_ready_o(x_mem_ready_o),
      .x_mem_req_i  (x_mem_req_i),
      .x_mem_resp_o (x_mem_resp_o),

      // Memory Result Interface
      .x_mem_result_valid_o(x_mem_result_valid_o),
      .x_mem_result_o(x_mem_result_o),

      // Result Interface
      .x_result_valid_i(x_result_valid_i),
      .x_result_ready_o(x_result_ready_o),
      .x_result_i(x_result_i),

      .irq_i    (irq_i),
      .irq_ack_o(irq_ack_o),
      .irq_id_o (irq_id_o),

      .debug_req_i      (debug_req_i),
      .debug_havereset_o(debug_havereset_o),
      .debug_running_o  (debug_running_o),
      .debug_halted_o   (debug_halted_o),

      .fetch_enable_i(fetch_enable_i),
      .core_sleep_o  (core_sleep_o)
  );

  // complete OBI configuration (obi_cfg_t) for Subordinate (core) and Manager (memoria)

  localparam obi_pkg::obi_cfg_t SbrPortObiCfg = '{
      UseRReady:   1'b0, // CV32E40PX does not use rready
      CombGnt:     1'b0,
      AddrWidth:   32,
      DataWidth:   32,
      IdWidth:     1,    // at least 1 bit to avoid vectors with width 0 in stream_fifo
      Integrity:   1'b0,
      BeFull:      1'b0,
      OptionalCfg: '{
          UseAtop:    1'b1, // mandatory 1'b1 on subordinate port
          UseMemtype: 1'b0,
          UseProt:    1'b0,
          UseDbg:     1'b0,
          AUserWidth: 0,
          WUserWidth: 0,
          RUserWidth: 0,
          MidWidth:   0,
          AChkWidth:  0,
          RChkWidth:  0
      }
  };

  localparam obi_pkg::obi_cfg_t MgrPortObiCfg = '{
      UseRReady:   1'b0,
      CombGnt:     1'b0,
      AddrWidth:   32,
      DataWidth:   32,
      IdWidth:     1,
      Integrity:   1'b0,
      BeFull:      1'b0,
      OptionalCfg: '{
          UseAtop:    1'b0, // mandatory 1'b0 on manager port
          UseMemtype: 1'b0,
          UseProt:    1'b0,
          UseDbg:     1'b0,
          AUserWidth: 0,
          WUserWidth: 0,
          RUserWidth: 0,
          MidWidth:   0,
          AChkWidth:  0,
          RChkWidth:  0
      }
  };

  // use macros to generate the structs sbr_port_obi_req_t, sbr_port_obi_rsp_t, ecc.
  `OBI_TYPEDEF_ALL(sbr_port_obi, SbrPortObiCfg)
  `OBI_TYPEDEF_ALL(mgr_port_obi, MgrPortObiCfg)

  sbr_port_obi_req_t sbr_port_req;
  sbr_port_obi_rsp_t sbr_port_rsp;
  mgr_port_obi_req_t mgr_port_req;
  mgr_port_obi_rsp_t mgr_port_rsp;

  // Mapping: Core <-> Subordinate port of obi_atop_resolver
  always_comb begin
    sbr_port_req                   = '0;
    sbr_port_req.req               = data_req_core;
    sbr_port_req.a.addr            = data_addr_core;
    sbr_port_req.a.we              = data_we_core;
    sbr_port_req.a.be              = data_be_core;
    sbr_port_req.a.wdata           = data_wdata_core;
    sbr_port_req.a.aid             = 1'b0; // ID = 0 for core (use 1'b1 for eventual TB requester)
    sbr_port_req.a.a_optional.atop = data_atop_core;
  end

  assign data_gnt_core    = sbr_port_rsp.gnt;
  assign data_rvalid_core = sbr_port_rsp.rvalid;
  assign data_rdata_core  = sbr_port_rsp.r.rdata;

  // Mapping: Manager port of obi_atop_resolver <-> external ports of top
  assign data_req_o   = mgr_port_req.req;
  assign data_addr_o  = mgr_port_req.a.addr;
  assign data_we_o    = mgr_port_req.a.we;
  assign data_be_o    = mgr_port_req.a.be;
  assign data_wdata_o = mgr_port_req.a.wdata;
  assign data_atop_o  = 6'b0; // atomics already resolved

  always_comb begin
    mgr_port_rsp         = '0;
    mgr_port_rsp.gnt     = data_gnt_i;
    mgr_port_rsp.rvalid  = data_rvalid_i;
    mgr_port_rsp.r.rdata = data_rdata_i;
    mgr_port_rsp.r.err   = 1'b0;
  end

  // Instanciate the obi_atop_resolver
  obi_atop_resolver #(
      .SbrPortObiCfg            (SbrPortObiCfg),
      .MgrPortObiCfg            (MgrPortObiCfg),
      .sbr_port_obi_req_t       (sbr_port_obi_req_t),
      .sbr_port_obi_rsp_t       (sbr_port_obi_rsp_t),
      .mgr_port_obi_req_t       (mgr_port_obi_req_t),
      .mgr_port_obi_rsp_t       (mgr_port_obi_rsp_t),
      .mgr_port_obi_a_optional_t(mgr_port_obi_a_optional_t),
      .mgr_port_obi_r_optional_t(mgr_port_obi_r_optional_t),
      .LrScEnable               (1'b1),
      .RegisterAmo              (1'b0)
    ) obi_atop_resolver_i (
      .clk_i         (clk_i),
      .rst_ni        (rst_ni),
      .testmode_i    (1'b0),
      .sbr_port_req_i(sbr_port_req),
      .sbr_port_rsp_o(sbr_port_rsp),
      .mgr_port_req_o(mgr_port_req),
      .mgr_port_rsp_i(mgr_port_rsp)
    );

  generate
    if (FPU) begin : fpu_gen

      assign apu_clk_en = apu_req | apu_busy;

      // FPU clock gate
      cv32e40px_clock_gate core_clock_gate_i (
          .clk_i       (clk_i),
          .en_i        (apu_clk_en),
          .scan_cg_en_i(scan_cg_en_i),
          .clk_o       (apu_clk)
      );

      // Instantiate the FPU wrapper
      cv32e40px_fp_wrapper #(
          .FPU_ADDMUL_LAT(FPU_ADDMUL_LAT),
          .FPU_OTHERS_LAT(FPU_OTHERS_LAT)
      ) fp_wrapper_i (
          .clk_i         (apu_clk),
          .rst_ni        (rst_ni),
          .apu_req_i     (apu_req),
          .apu_gnt_o     (apu_gnt),
          .apu_operands_i(apu_operands),
          .apu_op_i      (apu_op),
          .apu_flags_i   (apu_flags),
          .apu_rvalid_o  (apu_rvalid),
          .apu_rdata_o   (apu_rdata),
          .apu_rflags_o  (apu_rflags)
      );
    end else begin : no_fpu_gen
      // Drive FPU output signals to 0
      assign apu_gnt    = '0;
      assign apu_rvalid = '0;
      assign apu_rdata  = '0;
      assign apu_rflags = '0;
    end
  endgenerate

endmodule
