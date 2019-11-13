module Waveform(clk, btn, led, rgb, pinX, pinY);
    localparam integerBits    = 6;
    localparam fractionBits   = 25;
    localparam totalBits      = 1 + integerBits + fractionBits;
    localparam dtBits         = 20;
    localparam dtShift        = 32;
    localparam deltaSigmaBits = 16;
    
    localparam sigma = $rtoi(       10.0 * (2.0 ** fractionBits));
    localparam beta  = $rtoi((8.0 / 3.0) * (2.0 ** fractionBits));
    localparam rho   = $rtoi(       28.0 * (2.0 ** fractionBits));
    
    input wire clk;
    input wire [1 : 0] btn;
    output reg [3 : 0] led = 4'b0000;
    output reg [2 : 0] rgb = 3'b111;
    output wire pinX;
    output wire pinY;
    
    reg  signed [dtBits - 1 : 0] dt = 0.0001 * (2.0 ** dtShift);
    
    wire signed [totalBits - 1 : 0] x0;
    wire signed [totalBits - 1 : 0] y0;
    wire signed [totalBits - 1 : 0] x1;
    wire signed [totalBits - 1 : 0] y1;
    wire signed [totalBits - 1 : 0] x2;
    wire signed [totalBits - 1 : 0] y2;
    wire signed [totalBits - 1 : 0] x3;
    wire signed [totalBits - 1 : 0] y3;

    Lorenz #
    ( 
        .integerBits(integerBits),
        .fractionBits(fractionBits),
        .dtBits(dtBits),
        .dtShift(dtShift),
        .sigma(sigma),
        .beta(beta),
        .rho(rho)
    ) lorenz(.clk(clk), .dt(dt), .x(x0), .y(), .z(y0));

    Translate2 #(.bits(totalBits)) translate1(clk, x0, y0, 0, $rtoi(-23.5 * (2.0 ** fractionBits)), x1, y1);
    Scale2 #(.integerBits(integerBits), .fractionBits(fractionBits)) scale(clk, x1, y1, $rtoi(0.015 * (2.0 ** fractionBits)), $rtoi(0.015 * (2.0 ** fractionBits)), x2, y2);
    Translate2 #(.bits(totalBits)) translate2(clk, x2, y2, $rtoi(0.5 * (2.0 ** fractionBits)), $rtoi(0.5 * (2.0 ** fractionBits)), x3, y3);

    DeltaSigma #(.bits(deltaSigmaBits)) deltaSigmaX(clk, x3[totalBits - 1 : fractionBits - deltaSigmaBits], pinX);
    DeltaSigma #(.bits(deltaSigmaBits)) deltaSigmaY(clk, y3[totalBits - 1 : fractionBits - deltaSigmaBits], pinY);
endmodule


module Lorenz(clk, dt, x, y, z);
    parameter integerBits  = 6;
    parameter fractionBits = 25;
    parameter dtBits       = 16;
    parameter dtShift      = 32;
    parameter signed [integerBits + fractionBits : 0] sigma  =        10.0 * (2.0 ** fractionBits);
    parameter signed [integerBits + fractionBits : 0] beta   = (8.0 / 3.0) * (2.0 ** fractionBits);
    parameter signed [integerBits + fractionBits : 0] rho    =        28.0 * (2.0 ** fractionBits);
    
    localparam totalBits = 1 + integerBits + fractionBits;
    
    input  wire clk;
    input  wire signed [dtBits    - 1 : 0] dt;
    output reg  signed [totalBits - 1 : 0] x =  8.00 * (2.0 ** fractionBits);
    output reg  signed [totalBits - 1 : 0] y =  8.00 * (2.0 ** fractionBits);
    output reg  signed [totalBits - 1 : 0] z = 27.00 * (2.0 ** fractionBits);
    
    reg signed [totalBits * 2 - 1 : 0] dxdt = 0;
    reg signed [totalBits * 2 - 1 : 0] dydt = 0;
    reg signed [totalBits * 2 - 1 : 0] dzdt = 0;

    always @(posedge clk)
    begin
        dxdt = (sigma * (y - x)) >>> fractionBits;
        dydt = ((x * (rho - z)) >>> fractionBits) - y;
        dzdt = (x * y - beta * z) >>> fractionBits;

        x = x + ((dxdt * dt) >>> dtShift);
        y = y + ((dydt * dt) >>> dtShift);
        z = z + ((dzdt * dt) >>> dtShift);
    end
endmodule


module Scale2(clk, xIn, yIn, xScale, yScale, xOut, yOut);
    parameter integerBits   = 6;
    parameter fractionBits  = 25;

    localparam totalBits = 1 + integerBits + fractionBits;
    localparam multiplicationBits = totalBits + fractionBits;
    
    input  wire clk;
    input  wire signed [totalBits - 1 : 0] xIn;
    input  wire signed [totalBits - 1 : 0] yIn;
    input  wire signed [totalBits - 1 : 0] xScale;
    input  wire signed [totalBits - 1 : 0] yScale;
    output reg  signed [totalBits - 1 : 0] xOut = 0;
    output reg  signed [totalBits - 1 : 0] yOut = 0;
    
    wire signed [multiplicationBits - 1 : 0] x = (xIn * xScale) >>> fractionBits;
    wire signed [multiplicationBits - 1 : 0] y = (yIn * yScale) >>> fractionBits;

    always @(posedge clk)
    begin
        xOut <= x;
        yOut <= y;
    end
endmodule


module Translate2(clk, xIn, yIn, xTranslation, yTranslation, xOut, yOut);
    parameter bits = 32;
    
    input  wire clk;
    input  wire signed [bits - 1 : 0] xIn;
    input  wire signed [bits - 1 : 0] yIn;
    input  wire signed [bits - 1 : 0] xTranslation;
    input  wire signed [bits - 1 : 0] yTranslation;
    output reg  signed [bits - 1 : 0] xOut = 0;
    output reg  signed [bits - 1 : 0] yOut = 0;

    always @(posedge clk)
    begin
        xOut <= xIn + xTranslation;
        yOut <= yIn + yTranslation;
    end
endmodule


module DeltaSigma(clk, in, out);
    parameter bits = 32;

    input wire clk;
    input wire [bits - 1 : 0] in;
    output reg out = 0;
    
    reg [bits : 0] sum = 0;
    
    always @(posedge clk)
    begin
        sum = sum + in;
        out = sum[bits];
        sum[bits] = 0;
    end
endmodule
