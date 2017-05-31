//=================================================================
//=================================================================
//
// Filename: systemverilog_template.sv
//
//=================================================================
//=================================================================


//=================================================================
// Module template
//=================================================================

module nameofmodule
  #(parameter MPARAM1 = value1,
    parameter MPARAM2 = value2,
    parameter MPARAM3 = value3
   )
   (
    input                      clk,
    input                      rst_n,
    input                      scalarin1,
    input  wire                scalarin2,
    input        [MPARAM1-1:0] vectorin1,
    input        [MPARAM2-1:0] vectorin2,

    output logic               scalarout1,
    output logic               scalarout2,
    output logic [MPARAM3-1:0] vectorout1,
    output logic [MPARAM3-1:0] vectorout2
   );


   // Declarations
   //
   //  - local parameter declarations
   //  - local type declarations
   //  - local variable declarations
   //  - local function declarations
   //  - local task declarations
   //  - local interface declarations

   // Sub-module Instantiations
   //  - includes generate statements

   // Logic Functionality
   //
   //  - continuous assignments
   //  - procedural blocks
   //      - always_*
   //  - generate statements
   
endmodule // nameofmodule


//=================================================================
// always_comb/always_ff/always_latch templates
//=================================================================

  always_ff @(posedge clk, negedge reset) begin: seq_proc

    if (~reset) begin: reset_logic
      seq_sig1 <= '0;
      seq_sig2 <= '1;
    end: reset_logic

    else begin: sequential_assignment
      seq_sig1_lhs <= seq_sig1_rhs;
      seq_sig2_lhs <= seq_sig2_rhs;
    end: sequential_assignment

  end: seq_proc
     
  always_latch begin: lat_proc
    if (latch_enable_signal_or_expression) begin: lat_logic
       lat_sig_lhs <= lat_sig_rhs;
    end: lat_logic
  end: lat_proc
     
  always_comb begin: comb_proc
     comb_sig_lhs = comb_sig_rhs;
  end: comb_proc

//=================================================================
// Procedural Programming templates
//=================================================================

  // case statements
  //----------------
   unique case (<2-bit select>)
      2'b00  : begin
                  <statement>;
               end
      2'b01  : begin
                  <statement>;
               end
      2'b10  : begin
                  <statement>;
               end
      2'b11  : begin
                  <statement>;
               end
      default: begin
                  <statement>;
               end
   endcase
				
  // for loop
  //---------
   
  for (int i=0; i<LOOP_ITERATIONS; i++) begin: forloop
    // statements
  end: forloop

  // while loop
  //--------------
  while (i > NUM) begin
    i = i/2;
    // do something ...
  end

  // do-while loop
  //--------------
  // will execute at least once
    do begin
      // do something ...
      i -= 1;
    end
    while (i > NUM);

  // foreach loop
  //-------------
  foreach (array[]) begin
    //do something
  end
   
  // disable (Verilog), break, continue, return
  //-------------------------------------------


  // disable (Verilog)
  //------------------

  // find first bit set within a range of bits
  always @* begin
    begin: loop
      integer i;
      first_bit = 0;
      for (i=0; i<=63; i=i+1) begin: PASS
        if (i < start_range)
          disable PASS;         // continue loop
        if (i > end_range)
          disable loop;         // break out of loop
        if ( data[i] ) begin
          first_bit = i;
          disable loop;         // break out of loop
        end
      end // end of one PASS of loop
    end // end of the loop
    //... // process data based on first bit set
  end

  // continue, break
  //------------------

    for () begin
      if (condition)
        continue;
      // do something ...
    end

  // find first bit set within a range of bits
  always_comb begin
    first_bit = 0;
    for (int i=0; i<=63; i=i+1) begin
      if (i < start_range) continue;
      if (i > end_range)   break;  // exit loop
      if ( data[i] ) begin
        first_bit = i;
        break;  // exit loop
      end
    end // end of the loop
    //... // process data based on first bit set
  end

  // return
  //-------

   task task_name ();
     ...
     if (condition)
       return;          // exit task (note: no expression)
     ...
   endtask

   function function_name ();
     ...
     if (condition)
       return expression1;    // exit function (note: expression required)
     ...
     return expression2;      // exit function (note: expression required)
   endfunction

  // example
  task add_to_max (input  [ 5:0] max,
                   output [63:0] result);
    result = 1;
    if (max == 0) return; // exit task
    for (int i=1; i<=63; i=i+1) begin
      result = result + result;
      if (i == max) return; // exit task
    end
  endtask

  // if-then-else
  //-------------

  if (cond1) begin
    // statements
  end
  else if (cond2) begin
    // statements
  end
  else begin
    // statements
  end

   // compact style
   if (cond1) statement1;
   if (cond2) statement2;
   if (cond3) statement3;

   if      (cond1) statement1;
   else if (cond2) statement2;
   else if (cond3) statement3;
   else            statement4;
   

//=================================================================
// Generate Templates
//=================================================================

  genvar genI;
  generate
    for (genI=0; genI<GENNUM; genI+=1) begin: genloop
      // code to be generated
      assign ;
       
      module1_name module1_inst_name(.port1(),.port2());
      module2_name module2_inst_name(.port1(),.port2());

    end: genloop
  endgenerate

   genvar <var1>, <var2>;
   generate
      for (<var1>=0; <var1> < <limit>; <var1>=<var1>+1) 
      begin: <label_1>
         for (<var2>=0; <var2> < <limit>; <var2>=<var2>+1) 
         begin: <label_2>
            <code>
         end
      end
   endgenerate

   generate
      if (<condition>) begin: <label_1>
         <code>;
      end else if (<condition>) begin: <label_2>
         <code>;
      end else begin: <label_3>
         <code>;
      end
   endgenerate
//=================================================================
// FSM Templates
//=================================================================

  // FSM in single sequential block
  //-------------------------------
  typedef enum logic [2:0] {STATE0 = 3'b000,
                            STATE1 = 3'b001,
                            STATE2 = 3'b010} state_t;

  state_t curr_state;

  always_ff @(posedge clk, posedge reset) begin: seq_fsm

    if (reset) begin: reset_logic
      fsm_out_reg1 <= '0;
      fsm_out_reg2 <= '0;
      curr_state   <= STATE0;
    end: reset_logic

    else begin: seq_fsm_sequencer

      unique case (curr_state)

        STATE0: begin: state0_state
          fsm_out_reg1 <= '1;
          curr_state   <= STATE1;
        end: state0_state

        STATE1: begin: state1_state
          fsm_out_reg2 <= '1;
          curr_state   <= STATE2;
        end: state1_state

        STATE2: begin: state2_state
          fsm_out_reg2 <= '0;
          curr_state   <= STATE0;
        end: state2_state

	// I don't think this is needed
        default: begin: unknown_state
          curr_state <= STATE0;
          `ifndef SYNTHESIS // synthesis ignores this code
            $display("Unknown condition"); $finish();
          `endif
        end: unknown_state

      endcase
    end: seq_fsm_sequencer
  end: seq_fsm


  // mini FSM in single sequential block
  //-------------------------------
  typedef enum logic [2:0] {STATE0 = 3'b000,
                            STATE1 = 3'b001,
                            STATE2 = 3'b010} state_t;

  state_t curr_state;

  always_ff @(posedge clk, posedge reset) begin: mini_fsm
    if (reset) curr_state <= STATE0;
    else
      unique case (curr_state)
        STATE0: if (cond1) curr_state <= STATE1;
        STATE1: if (cond2) curr_state <= STATE2;
        STATE2: if (cond3) curr_state <= STATE0;
      endcase // unique case (curr_state)
  end: mini_fsm
   
   
  // FSM using sequential and combo blocks
  //--------------------------------------

  typedef enum logic [2:0] {STATE0 = 3'b000,
                            STATE1 = 3'b001,
                            STATE2 = 3'b010} state_t;

  state_t curr_state, next_state;

  always_ff @(posedge clk, posedge reset) begin: fsm_seq
    if (reset) curr_state <= STATE0;
    else       curr_state <= next_state;
  end: fsm_seq

  always_comb begin: fsm_combo
    next_state = curr_state;
    unique case (curr_state)

      STATE0: begin: state0_state
        fsm_out_com1 = '1;
        curr_state   = STATE1;
      end: state0_state

      STATE1: begin: state1_state
        fsm_out_com2 = '1;
        curr_state   = STATE2;
      end: state1_state

      STATE2: begin: state2_state
        fsm_out_com2 = '0;
        curr_state   = STATE0;
      end: state2_state

      // I don't think this is needed
      default: begin: unknown_state
        State = STATE0;
      end: unknown_state

    endcase // unique case (curr_state)
  end // block: fsm_combo


  // One-hot FSM
  //------------

  enum {STATE1_BIT = 0,
        STATE2_BIT = 1,
        STATE3_BIT = 2} state_bit;

  typedef enum logic [2:0] {STATE1 = 3'b001<<STATE1_BIT,
                            STATE2 = 3'b001<<STATE2_BIT,
                            STATE3 = 3'b001<<STATE3_BIT} state_t;

//  typedef enum logic [2:0] {STATE1 = 3'b001,
//                            STATE2 = 3'b010,
//                            STATE3 = 3'b100} state_t;

  state_t State, Next;

  always_ff @(posedge clock, negedge resetN)
    if (!resetN) State <= STATE1;
    else         State <= Next;

  always_comb begin: set_next_state
    Next = State;
    unique case (1'b1)  // reversed case statement
      State[STATE1_BIT]: if (cond1) Next = STATE2;
      State[STATE2_BIT]: if (cond2) Next = STATE3;
      State[STATE3_BIT]: if (cond3) Next = STATE1;
    endcase
  end: set_next_state

  always_comb begin: set_outputs
    {out1, out2, out3} = 3'b000;
    unique case (1'b1)  // reversed case statement
      State[STATE1_BIT]: out1 = 1;
      State[STATE2_BIT]: out2 = 1;
      State[STATE3_BIT]: out3 = 1;
    endcase
  end: set_outputs



//=================================================================
// Interface template
//=================================================================

interface nameofinterface
  #(parameter IPARAM1 = value1,
    parameter IPARAM2 = value2
   )
   (
    input  logic clk,
    input  logic rst_n,

    output logic scalarout1,
   );

  // type declarations
  typedef ...

  // signal declarations
  logic 	 isig1, isig2;
  logic 	 osig1, osig2;
   
  logic        scalar1;
  logic        scalar2;
  logic [ 7:0] vector1;

  // Modport definitions
  modport modport1 (
    input  isig1, isig2,
    output osig1, osig2
  );

  modport modport2 (
    input  osig1, osig2,
    output isig1, isig2
  );

   // Task definitions
   task nameoftask ();
   endtask // nameoftask

   // Function definitions
   function type nameoffunc ();
   endfunction // nameoffunc
   
   // Procedural blocks and assignments
   initial begin end

   always_comb begin end

   always_ff @() begin end

   assign ;

endinterface

//=================================================================
// Function template
//
// Note: For synthesis, use void functions instead of tasks
//=================================================================

  function automatic int log2 (input int n);
    if (n <=1) return 1;  // abort function
    log2 = 0;
    while (n > 1) begin
      n = n/2;
      log2++;
    end
    return log2;
  endfunction

//=================================================================
// Task Template
//=================================================================


