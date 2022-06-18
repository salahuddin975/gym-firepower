initial_model = '''
scalar M1             /1e+8/,
       M2             /1e+5/,
       epsilon        /1e-2/;


set i               "Set of nodes according to the topology of the distribution network" /0*23/;
alias(i,j);
* set v               "Set of various generators"                                          ;

set c               "Load divided based on criticality"                                  /1*2/;

parameter                PGLbarT(i)       "Power generation lower limit",
                         PGUbarT(i)       "Power generation upper limit",
                         ThetaLbar(i)       "Power angle lower limit",
                         ThetaUbar(i)       "Power angle upper limit",
                         B(i,i)             "Admittance matrix",
                         PLoad(i)           "Load value",
                         PLbar(i,i)         "Line Flow Limits",
                         LineStat(i,i)      "Line Status",
                         Rampbar(i)       "Ramp rate limit",
                         IntDur             "Duration of a typical interval",
                         CritFrac(i,c)      "Fraction of loads critical and non-critical",
                         CritVal(c)         "Value of the critical load",
                         PGBegin(i)       "Generation at the begining";


$if not set gdxincname $abort 'no include file name for data file provided'
$gdxin %gdxincname%
$load PGLbarT, PGUbarT, ThetaLbar, ThetaUbar, B, PLoad, PLbar, LineStat, Rampbar, IntDur, CritFrac, CritVal, PGBegin
$gdxin


binary variable   OutGen(i)            "Generator operating condition";

variable          Theta(i)             "Power angle",
                  PGn(i)               "generation",
                  DPLoadC(i)           "Critical Load Not Served",
                  DPLoadNC(i)          "Non-Critical Load Not Served",
                  LineFlow(i,j)        "Line Flow",
                  z                      "The objective funcion value";

* Graph Partitioning Constraints
equations        Obj           "The equation representing the objective function",
                 GenFlow(i),
                 BusLineFlow1(i,j),
                 BusLineFlow2(i,j),
                 DCFlowBal(i,j);


Obj..                    z =e= sum(i, CritVal('1')*DPLoadC(i) + CritVal('2')*DPLoadNC(i)) +  epsilon*sum(i,(PGn(i) - PGBegin(i))*(PGn(i) - PGBegin(i)));
GenFlow(i)..           PGn(i) - PLoad(i) + DPLoadC(i) + DPLoadNC(i) - sum(j,LineFlow(i,j)) =e= 0;
BusLineFlow1(i,j)..    LineFlow(i,j) - (Theta(i) - Theta(j))*B(i,j)  + (1 - LineStat(i,j))*M2 =g= 0;
BusLineFlow2(i,j)..    LineFlow(i,j) - (Theta(i) - Theta(j))*B(i,j)  - (1 - LineStat(i,j))*M2 =l= 0;
Theta.lo(i) = ThetaLbar(i);
Theta.up(i) = ThetaUbar(i);
DCFlowBal(i,j)..       LineFlow(i,j) + LineFlow(j,i) =e= 0;
DPLoadC.lo(i) = 0;
DPLoadC.up(i) = PLoad(i)*CritFrac(i,'1');
DPLoadNC.lo(i) = 0;
DPLoadNC.up(i) = PLoad(i)*CritFrac(i,'2');

equations        LineFlowLim1(i,j)           "Line Flow Status",
                 LineFlowLim2(i,j)           "Line Flow Status";

* Line Related Constraint
LineFlowLim1(i,j)..    LineFlow(i,j) =g= -1*PLbar(i,j)*LineStat(i,j);
LineFlowLim2(i,j)..    LineFlow(i,j) =l= PLbar(i,j)*LineStat(i,j);


equations        GenOpt1(i)                    "Generator operating condition",
                 GenOpt2(i)                    "Generator operating condition";

* Generator Related Constraint
GenOpt1(i)..    PGn(i) =g= PGLbarT(i);
GenOpt2(i)..    PGn(i) =l= PGUbarT(i);


model PART /all/;
PART.nodLim = 500;
PART.domlim  = 10000;

option qcp = conopt;
solve PART minimizing z using qcp

* execute_unload 'soln01', Theta;

parameter flow(i,j), flowP(i,j), flow2(i,j), ZVal, ZValP(i), PGn_total, Load_loss, DPLoad(i), PLoad_(i), p_solved_nc, p_solved_c, p_solved;
flow(i,j)  = LineFlow.l(i,j) - (Theta.l(i) - Theta.l(j))*B(i,j) + (1 - LineStat(i,j))*M2;
flowP(i,j) = LineFlow.l(i,j) - (Theta.l(i) - Theta.l(j))*B(i,j) - (1 - LineStat(i,j))*M2;
flow2(i,j) =  flow(i,j) + flow(j,i);

ZVal    = sum(i, CritVal('1')*DPLoadC.l(i) + CritVal('2')*DPLoadNC.l(i));
ZValP(i) = CritVal('1')*DPLoadC.l(i) + CritVal('2')*DPLoadNC.l(i);
PGn_total = sum(i, PGn.l(i));
DPLoad(i) = DPLoadC.l(i) + DPLoadNC.l(i);
PLoad_(i) = PLoad(i) - DPLoad(i); 
Load_loss = sum(i, DPLoad(i));
p_solved_c = sum(i, PLoad(i)*CritFrac(i,'1')) -  sum(i, DPLoadC.l(i));
p_solved_nc = sum(i, PLoad(i)*CritFrac(i,'2')) - sum(i, DPLoadNC.l(i));
p_solved = sum(i, PLoad_(i));
parameter ModStat        "Model status";
ModStat = PART.modelstat;

display  ModStat, ZVal, ZValP, Pgn_total, DPLoad, PLoad_, Load_loss, p_solved_nc, p_solved_c;

'''

run_time_model = '''
scalar M1             /1e+8/,
       M2             /1e+5/,
       epsilon        /1e-2/;


set i               "Set of nodes according to the topology of the distribution network" /0*23/;
alias(i,j);
* set v               "Set of various generators"                                          /1*1/;

set c               "Load divided based on criticality"                                  /1*2/;

parameter                PGLbarT(i)       "Power generation lower limit",
                         PGUbarT(i)       "Power generation upper limit",
                         ThetaLbar(i)       "Power angle lower limit",
                         ThetaUbar(i)       "Power angle upper limit",
                         B(i,i)             "Admittance matrix",
                         PLoad(i)           "Load value",
                         PLbar(i,i)         "Line Flow Limits",
                         LineStat(i,i)      "Line Status",
                         Rampbar(i)       "Ramp rate limit",
                         IntDur             "Duration of a typical interval",
                         CritFrac(i,c)      "Fraction of loads critical and non-critical",
                         CritVal(c)         "Value of the critical load",
                         PGBegin(i)       "Generation at the begining";


$if not set gdxincname $abort 'no include file name for data file provided'
$gdxin %gdxincname%
$load PGLbarT, PGUbarT, ThetaLbar, ThetaUbar, B, PLoad, PLbar, LineStat, Rampbar, IntDur, CritFrac, CritVal, PGBegin
$gdxin

binary variable   OutGen(i)            "Generator operating condition";

variable          Theta(i)             "Power angle",
                  PGn(i)               "generation",
                  DPLoadC(i)           "Critical Load Not Served",
                  DPLoadNC(i)          "Non-Critical Load Not Served",
                  LineFlow(i,j)        "Line Flow",
                  z                      "The objective funcion value";

* Graph Partitioning Constraints
equations        Obj           "The equation representing the objective function",
                 GenFlow(i),
                 BusLineFlow1(i,j),
                 BusLineFlow2(i,j),
                 DCFlowBal(i,j);


Obj..                    z =e= sum(i, CritVal('1')*DPLoadC(i) + CritVal('2')*DPLoadNC(i)) + epsilon*sum(i, (PGn(i) - PGBegin(i))*(PGn(i) - PGBegin(i)));
GenFlow(i)..           PGn(i) - PLoad(i) + DPLoadC(i) + DPLoadNC(i) - sum(j,LineFlow(i,j)) =e= 0;
BusLineFlow1(i,j)..    LineFlow(i,j) - (Theta(i) - Theta(j))*B(i,j)  + (1 - LineStat(i,j))*M2 =g= 0;
BusLineFlow2(i,j)..    LineFlow(i,j) - (Theta(i) - Theta(j))*B(i,j)  - (1 - LineStat(i,j))*M2 =l= 0;
Theta.lo(i) = ThetaLbar(i);
Theta.up(i) = ThetaUbar(i);
DCFlowBal(i,j)..       LineFlow(i,j) + LineFlow(j,i) =e= 0;
DPLoadC.lo(i) = 0;
DPLoadC.up(i) = PLoad(i)*CritFrac(i,'1');
DPLoadNC.lo(i) = 0;
DPLoadNC.up(i) = PLoad(i)*CritFrac(i,'2');

equations        LineFlowLim1(i,j)           "Line Flow Status",
                 LineFlowLim2(i,j)           "Line Flow Status";

* Line Related Constraint
LineFlowLim1(i,j)..    LineFlow(i,j) =g= -1*PLbar(i,j)*LineStat(i,j);
LineFlowLim2(i,j)..    LineFlow(i,j) =l= PLbar(i,j)*LineStat(i,j);


equations        GenOpt1(i)                    "Generator operating condition",
                 GenOpt2(i)                    "Generator operating condition",
                 RRateLimit1(i)                "Generator output based on ramp rate",
                 RRateLimit2(i)                "Generator output based on ramp rate";

* Generator Related Constraint
GenOpt1(i)..    PGn(i) =g= PGLbarT(i);
GenOpt2(i)..    PGn(i) =l= PGUbarT(i);
RRateLimit1(i).. PGn(i) - PGBegin(i)  =l=  IntDur*Rampbar(i);
RRateLimit2(i).. PGn(i) - PGBegin(i)  =g= -IntDur*Rampbar(i);
*PGn.fx(i) = PGBegin(i);


model PART /all/;
PART.nodLim = 500;
PART.domlim  = 10000;
*PART.optfile = 1;

option qcp = conopt;
solve PART minimizing z using qcp

* execute_unload 'soln01', Theta;

parameter flow(i,j), flowP(i,j), flow2(i,j), ZVal, ZValP(i), PGn_total, Load_loss, DPLoad(i), PLoad_(i), p_solved_nc, p_solved_c, p_solved;
flow(i,j)  = LineFlow.l(i,j) - (Theta.l(i) - Theta.l(j))*B(i,j) + (1 - LineStat(i,j))*M2;
flowP(i,j) = LineFlow.l(i,j) - (Theta.l(i) - Theta.l(j))*B(i,j) - (1 - LineStat(i,j))*M2;
flow2(i,j) =  flow(i,j) + flow(j,i);

ZVal    = sum(i, CritVal('1')*DPLoadC.l(i) + CritVal('2')*DPLoadNC.l(i));
ZValP(i) = CritVal('1')*DPLoadC.l(i) + CritVal('2')*DPLoadNC.l(i);
PGn_total = sum(i, PGn.l(i));
DPLoad(i) = DPLoadC.l(i) + DPLoadNC.l(i);
PLoad_(i) = PLoad(i) - DPLoad(i); 
Load_loss = sum(i, DPLoad(i));
p_solved_c = sum(i, PLoad(i)*CritFrac(i,'1')) -  sum(i, DPLoadC.l(i));
p_solved_nc = sum(i, PLoad(i)*CritFrac(i,'2')) - sum(i, DPLoadNC.l(i));
p_solved = sum(i, PLoad_(i));
parameter ModStat        "Model status";
ModStat = PART.modelstat;

display  ModStat, ZVal, ZValP, Pgn_total, DPLoad, PLoad_, Load_loss, p_solved_nc, p_solved_c;
'''

initial_model_v2 = '''
scalar M1             /1e+9/,
       M2             /1e+5/,
       epsilon        /1e-2/;


set i               "Set of nodes according to the topology of the distribution network" /0*23/;
alias(i,j);
* set v               "Set of various generators"                                          /1*1/;

set c               "Load divided based on criticality"                                  /1*2/;

parameter                PGLbarT(i)       "Power generation lower limit",
                         PGUbarT(i)       "Power generation upper limit",
                         ThetaLbar(i)       "Power angle lower limit",
                         ThetaUbar(i)       "Power angle upper limit",
                         B(i,i)             "Admittance matrix",
                         PLoad(i)           "Load value",
                         PLbar(i,i)         "Line Flow Limits",
                         LineStat(i,i)      "Line Status",
                         NodeStat(i)      "Node Status",
                         GenStat(i)       "Generator Status",
                         Rampbar(i)       "Ramp rate limit",
                         IntDur             "Duration of a typical interval",
                         CritFrac(i,c)      "Fraction of loads critical and non-critical",
                         CritVal(c)         "Value of the critical load",
                         PGBegin(i)       "Generation at the begining";


$if not set gdxincname $abort 'no include file name for data file provided'
$gdxin %gdxincname%
$load PGLbarT, PGUbarT, ThetaLbar, ThetaUbar, B, PLoad, PLbar, LineStat, NodeStat, GenStat, Rampbar, IntDur, CritFrac, CritVal, PGBegin
$gdxin


binary variable   OutGen(i)            "Generator operating condition";

variable          Theta(i)             "Power angle",
                  PGn(i)               "generation",
                  DPLoadC(i)           "Critical Load Not Served",
                  DPLoadNC(i)          "Non-Critical Load Not Served",
                  LineFlow(i,j)        "Line Flow",
                  z                      "The objective funcion value";

* Graph Partitioning Constraints
equations        Obj           "The equation representing the objective function",
                 GenFlow(i),
                 BusLineFlow1(i,j);
*                 BusLineFlow2(i,j),
*                 DCFlowBal(i,j);


Obj..                    z =e= sum(i, CritVal('1')*DPLoadC(i) + CritVal('2')*DPLoadNC(i)) - epsilon*sum(i,OutGen(i));
GenFlow(i)..           PGn(i) - PLoad(i)*NodeStat(i) + DPLoadC(i) + DPLoadNC(i) - sum(j,LineFlow(i,j)) =e= 0;
BusLineFlow1(i,j)..    LineFlow(i,j) - (Theta(i) - Theta(j))*B(i,j)*LineStat(i,j)  =e= 0;
Theta.lo(i) = ThetaLbar(i);
Theta.up(i) = ThetaUbar(i);
DPLoadC.lo(i) = 0;
DPLoadC.up(i) = PLoad(i)*CritFrac(i,'1')*NodeStat(i);
DPLoadNC.lo(i) = 0;
DPLoadNC.up(i) = PLoad(i)*CritFrac(i,'2')*NodeStat(i);

equations        LineFlowLim1(i,j)           "Line Flow Status",
                 LineFlowLim2(i,j)           "Line Flow Status";

* Line Related Constraint
LineFlowLim1(i,j)..    LineFlow(i,j) =g= -1*PLbar(i,j)*LineStat(i,j);
LineFlowLim2(i,j)..    LineFlow(i,j) =l= PLbar(i,j)*LineStat(i,j);


equations        GenOpt1(i)                    "Generator operating condition",
                 GenOpt2(i)                    "Generator operating condition";

* Generator Related Constraint
GenOpt1(i)..    PGn(i) =g= PGLbarT(i)*NodeStat(i)*OutGen(i);
GenOpt2(i)..    PGn(i) =l= PGUbarT(i)*NodeStat(i)*OutGen(i);
OutGen.up(i) = GenStat(i);

model PART /all/;
PART.nodLim = 500;
PART.domlim  = 10000;

option miqcp = scip;
solve PART minimizing z using miqcp

* execute_unload 'soln01', Theta;

parameter flow(i,j), flowP(i,j), flow2(i,j), ZVal, ZValP(i), PGn_total, Load_loss, DPLoad(i), PLoad_(i), p_solved_nc, p_solved_c, p_solved;
variable OutGen_val(i), DPLoad_val(i), PLoad_served(i);
flow(i,j)  = LineFlow.l(i,j) - (Theta.l(i) - Theta.l(j))*B(i,j) + (1 - LineStat(i,j))*M2;
flowP(i,j) = LineFlow.l(i,j) - (Theta.l(i) - Theta.l(j))*B(i,j) - (1 - LineStat(i,j))*M2;
flow2(i,j) =  flow(i,j) + flow(j,i);

ZVal    = sum(i, CritVal('1')*DPLoadC.l(i) + CritVal('2')*DPLoadNC.l(i));
ZValP(i) = CritVal('1')*DPLoadC.l(i) + CritVal('2')*DPLoadNC.l(i);
PGn_total = sum(i, PGn.l(i));
DPLoad(i) = DPLoadC.l(i) + DPLoadNC.l(i);
PLoad_(i) = PLoad(i) - DPLoad(i); 
Load_loss = sum(i, DPLoad(i));
p_solved_c = sum(i, PLoad(i)*CritFrac(i,'1')) -  sum(i, DPLoadC.l(i));
p_solved_nc = sum(i, PLoad(i)*CritFrac(i,'2')) - sum(i, DPLoadNC.l(i));
p_solved = sum(i, PLoad_(i));
OutGen_val.lo(i) = 0;
OutGen_val.up(i) = 0;
OutGen_val.l(i) = 0;
OutGen_val.fx(i) = OutGen.l(i);
DPLoad_val.l(i) = 0;
DPLoad_val.lo(i) = 0;
DPLoad_val.up(i) = 0;
DPLoad_val.fx(i) = DPLoad(i);
PLoad_served.l(i) = 0;
PLoad_served.lo(i) = 0;
PLoad_served.up(i) = 0;
PLoad_served.fx(i) = PLoad_(i);
parameter ModStat        "Model status";
ModStat = PART.modelstat;

display  ModStat, ZVal, ZValP, Pgn_total, DPLoad, PLoad_, Load_loss, Theta.l, LineFlow.l;


'''

run_time_model_v2 = '''
scalar M1             /1e+8/,
       M2             /1e+5/,
       epsilon        /1e-2/;


set i               "Set of nodes according to the topology of the distribution network" /0*23/;
alias(i,j);
* set v               "Set of various generators"                                          /1*1/;

set c               "Load divided based on criticality"                                  /1*2/;

parameter                PGLbarT(i)       "Power generation lower limit",
                         PGUbarT(i)       "Power generation upper limit",
                         ThetaLbar(i)     "Power angle lower limit",
                         ThetaUbar(i)     "Power angle upper limit",
                         B(i,i)           "Admittance matrix",
                         PLoad(i)         "Load value",
                         PLbar(i,i)       "Line Flow Limits",
                         LineStat(i,i)    "Line Status",
                         NodeStat(i)      "Node Status",
                         GenStat(i)       "Generator Status",
                         Rampbar(i)       "Ramp rate limit",
                         IntDur           "Duration of a typical interval",
                         CritFrac(i,c)    "Fraction of loads critical and non-critical",
                         CritVal(c)       "Value of the critical load",
                         PGBegin(i)       "Generation at the begining";


$if not set gdxincname $abort 'no include file name for data file provided'
$gdxin %gdxincname%
$load PGLbarT, PGUbarT, ThetaLbar, ThetaUbar, B, PLoad, PLbar, LineStat, NodeStat, GenStat, Rampbar, IntDur, CritFrac, CritVal, PGBegin
$gdxin

binary variable   OutGen(i)            "Generator operating condition",
                  OutGen1(i)            "Generator operating condition";

variable          Theta(i)             "Power angle",
                  PGn(i)               "generation",
                  DPLoadC(i)           "Critical Load Not Served",
                  DPLoadNC(i)          "Non-Critical Load Not Served",
                  LineFlow(i,j)        "Line Flow",
                  z                      "The objective funcion value";

equations        Obj           "The equation representing the objective function",
                 GenFlow(i),
                 BusLineFlow1(i,j);


Obj..                    z =e= sum(i, CritVal('1')*DPLoadC(i) + CritVal('2')*DPLoadNC(i)) +  epsilon*sum(i,(PGn(i) - PGBegin(i))*(PGn(i) - PGBegin(i)));
GenFlow(i)..           PGn(i) - PLoad(i)*NodeStat(i) + DPLoadC(i) + DPLoadNC(i) - sum(j,LineFlow(i,j)) =e= 0;
BusLineFlow1(i,j)..    LineFlow(i,j) - (Theta(i) - Theta(j))*B(i,j)*LineStat(i,j)  =e= 0;
Theta.lo(i) = ThetaLbar(i);
Theta.up(i) = ThetaUbar(i);
DPLoadC.lo(i) = 0;
DPLoadC.up(i) = PLoad(i)*CritFrac(i,'1')*NodeStat(i);
DPLoadNC.lo(i) = 0;
DPLoadNC.up(i) = PLoad(i)*CritFrac(i,'2')*NodeStat(i);

equations        LineFlowLim1(i,j)           "Line Flow Status",
                 LineFlowLim2(i,j)           "Line Flow Status";

* Line Related Constraint
LineFlowLim1(i,j)..    LineFlow(i,j) =g= -PLbar(i,j)*LineStat(i,j);
LineFlowLim2(i,j)..    LineFlow(i,j) =l= PLbar(i,j)*LineStat(i,j);


equations        GenOpt1(i)                    "Generator operating condition",
                 GenOpt2(i)                    "Generator operating condition",
                 RRateLimit1(i)                "Generator output based on ramp rate",
                 RRateLimit2(i)                "Generator output based on ramp rate";

* Generator Related Constraint
GenOpt1(i)..    PGn(i) =g= PGLbarT(i)*NodeStat(i)*OutGen(i);
GenOpt2(i)..    PGn(i) =l= PGUbarT(i)*NodeStat(i)*OutGen(i);
RRateLimit1(i).. PGn(i) - PGBegin(i)  =l=  IntDur*Rampbar(i)*NodeStat(i)*OutGen(i);
RRateLimit2(i).. PGn(i) - PGBegin(i)  =g= -IntDur*Rampbar(i)*NodeStat(i)*OutGen(i) - PGUbarT(i)*(1 - OutGen(i));
OutGen.up(i) = GenStat(i);



model PART /all/;
* PART.nodLim = 500;
* PART.domlim  = 10000;
*PART.optfile = 1;
PART.optcr = 0.0001

option miqcp = scip;
solve PART minimizing z using miqcp

* execute_unload 'soln01', Theta;

parameter flow(i,j), flowP(i,j), flow2(i,j), ZVal, ZValP(i), PGn_total, Load_loss, DPLoad(i), PLoad_(i), p_solved_nc, p_solved_c, p_solved;
variable OutGen_val(i), DPLoad_val(i), PLoad_served(i);
flow(i,j)  = LineFlow.l(i,j) - (Theta.l(i) - Theta.l(j))*B(i,j) + (1 - LineStat(i,j))*M2;
flowP(i,j) = LineFlow.l(i,j) - (Theta.l(i) - Theta.l(j))*B(i,j) - (1 - LineStat(i,j))*M2;
flow2(i,j) =  flow(i,j) + flow(j,i);

ZVal    = sum(i, CritVal('1')*DPLoadC.l(i) + CritVal('2')*DPLoadNC.l(i));
ZValP(i) = CritVal('1')*DPLoadC.l(i) + CritVal('2')*DPLoadNC.l(i);
PGn_total = sum(i, PGn.l(i));
DPLoad(i) = DPLoadC.l(i) + DPLoadNC.l(i);
PLoad_(i) = PLoad(i) - DPLoad(i); 
Load_loss = sum(i, DPLoad(i));
p_solved_c = sum(i, PLoad(i)*CritFrac(i,'1')) -  sum(i, DPLoadC.l(i));
p_solved_nc = sum(i, PLoad(i)*CritFrac(i,'2')) - sum(i, DPLoadNC.l(i));
p_solved = sum(i, PLoad_(i));
OutGen_val.lo(i) = 0;
OutGen_val.up(i) = 0;
OutGen_val.l(i) = 0;
OutGen_val.fx(i) = OutGen.l(i);
DPLoad_val.l(i) = 0;
DPLoad_val.lo(i) = 0;
DPLoad_val.up(i) = 0;
DPLoad_val.fx(i) = DPLoad(i);
PLoad_served.l(i) = 0;
PLoad_served.lo(i) = 0;
PLoad_served.up(i) = 0;
PLoad_served.fx(i) = PLoad_(i);
parameter ModStat        "Model status";
ModStat = PART.modelstat;

display  ModStat, ZVal, ZValP, Pgn_total, DPLoad, PLoad_, Load_loss, Theta.l, LineFlow.l;

'''



initial_model_generation_rejection = '''
scalar M1             /1e+9/,
       M2             /1e+5/,
       epsilon        /1e-2/;


set i               "Set of nodes according to the topology of the distribution network" /0*23/;
alias(i,j);
* set v               "Set of various generators"                                          /1*1/;

set c               "Load divided based on criticality"                                  /1*2/;

parameter                PGLbarT(i)       "Power generation lower limit",
                         PGUbarT(i)       "Power generation upper limit",
                         ThetaLbar(i)       "Power angle lower limit",
                         ThetaUbar(i)       "Power angle upper limit",
                         B(i,i)             "Admittance matrix",
                         PLoad(i)           "Load value",
                         PLbar(i,i)         "Line Flow Limits",
                         LineStat(i,i)      "Line Status",
                         NodeStat(i)      "Node Status",
                         GenStat(i)       "Generator Status",
                         Rampbar(i)       "Ramp rate limit",
                         IntDur             "Duration of a typical interval",
                         CritFrac(i,c)      "Fraction of loads critical and non-critical",
                         CritVal(c)         "Value of the critical load",
                         PGBegin(i)       "Generation at the begining";


$if not set gdxincname $abort 'no include file name for data file provided'
$gdxin %gdxincname%
$load PGLbarT, PGUbarT, ThetaLbar, ThetaUbar, B, PLoad, PLbar, LineStat, NodeStat, GenStat, Rampbar, IntDur, CritFrac, CritVal, PGBegin
$gdxin


binary variable   OutGen(i)            "Generator operating condition";

variable          Theta(i)             "Power angle",
                  PGn(i)               "generation",
                  DPLoadC(i)           "Critical Load Not Served",
                  DPLoadNC(i)          "Non-Critical Load Not Served",
                  LineFlow(i,j)        "Line Flow",
                  z                      "The objective funcion value";

* Graph Partitioning Constraints
equations        Obj           "The equation representing the objective function",
                 GenFlow(i),
                 BusLineFlow1(i,j);
*                 BusLineFlow2(i,j),
*                 DCFlowBal(i,j);


Obj..                    z =e= sum(i, CritVal('1')*DPLoadC(i) + CritVal('2')*DPLoadNC(i)) - epsilon*sum(i,OutGen(i));
GenFlow(i)..           PGn(i) - PLoad(i)*NodeStat(i) + DPLoadC(i) + DPLoadNC(i) - sum(j,LineFlow(i,j)) =e= 0;
BusLineFlow1(i,j)..    LineFlow(i,j) - (Theta(i) - Theta(j))*B(i,j)*LineStat(i,j)  =e= 0;
Theta.lo(i) = ThetaLbar(i);
Theta.up(i) = ThetaUbar(i);
DPLoadC.lo(i) = 0;
DPLoadC.up(i) = PLoad(i)*CritFrac(i,'1')*NodeStat(i);
DPLoadNC.lo(i) = 0;
DPLoadNC.up(i) = PLoad(i)*CritFrac(i,'2')*NodeStat(i);

equations        LineFlowLim1(i,j)           "Line Flow Status",
                 LineFlowLim2(i,j)           "Line Flow Status";

* Line Related Constraint
LineFlowLim1(i,j)..    LineFlow(i,j) =g= -1*PLbar(i,j)*LineStat(i,j);
LineFlowLim2(i,j)..    LineFlow(i,j) =l= PLbar(i,j)*LineStat(i,j);


equations        GenOpt1(i)                    "Generator operating condition",
                 GenOpt2(i)                    "Generator operating condition";

* Generator Related Constraint
GenOpt1(i)..    PGn(i) =g= PGLbarT(i)*NodeStat(i)*OutGen(i);
GenOpt2(i)..    PGn(i) =l= PGUbarT(i)*NodeStat(i)*OutGen(i);
OutGen.up(i) = GenStat(i);

model PART /all/;
PART.nodLim = 500;
PART.domlim  = 10000;

option miqcp = scip;
solve PART minimizing z using miqcp

* execute_unload 'soln01', Theta;

parameter flow(i,j), flowP(i,j), flow2(i,j), ZVal, ZValP(i), PGn_total, Load_loss, DPLoad(i), PLoad_(i), p_solved_nc, p_solved_c, p_solved;
variable OutGen_val(i), DPLoad_val(i), PLoad_served(i);
flow(i,j)  = LineFlow.l(i,j) - (Theta.l(i) - Theta.l(j))*B(i,j) + (1 - LineStat(i,j))*M2;
flowP(i,j) = LineFlow.l(i,j) - (Theta.l(i) - Theta.l(j))*B(i,j) - (1 - LineStat(i,j))*M2;
flow2(i,j) =  flow(i,j) + flow(j,i);

ZVal    = sum(i, CritVal('1')*DPLoadC.l(i) + CritVal('2')*DPLoadNC.l(i));
ZValP(i) = CritVal('1')*DPLoadC.l(i) + CritVal('2')*DPLoadNC.l(i);
PGn_total = sum(i, PGn.l(i));
DPLoad(i) = DPLoadC.l(i) + DPLoadNC.l(i);
PLoad_(i) = PLoad(i) - DPLoad(i); 
Load_loss = sum(i, DPLoad(i));
p_solved_c = sum(i, PLoad(i)*CritFrac(i,'1')) -  sum(i, DPLoadC.l(i));
p_solved_nc = sum(i, PLoad(i)*CritFrac(i,'2')) - sum(i, DPLoadNC.l(i));
p_solved = sum(i, PLoad_(i));
OutGen_val.lo(i) = 0;
OutGen_val.up(i) = 0;
OutGen_val.l(i) = 0;
OutGen_val.fx(i) = OutGen.l(i);
DPLoad_val.l(i) = 0;
DPLoad_val.lo(i) = 0;
DPLoad_val.up(i) = 0;
DPLoad_val.fx(i) = DPLoad(i);
PLoad_served.l(i) = 0;
PLoad_served.lo(i) = 0;
PLoad_served.up(i) = 0;
PLoad_served.fx(i) = PLoad_(i);
parameter ModStat        "Model status";
ModStat = PART.modelstat;

display  ModStat, ZVal, ZValP, Pgn_total, DPLoad, PLoad_, Load_loss, Theta.l, LineFlow.l;


'''

run_time_model_generation_rejection = '''
scalar M1             /1e+8/,
       M2             /1e+5/,
       epsilon        /1e-2/;


set i               "Set of nodes according to the topology of the distribution network" /0*23/;
alias(i,j);
* set v               "Set of various generators"                                          /1*1/;

set c               "Load divided based on criticality"                                  /1*2/;

parameter                PGLbarT(i)       "Power generation lower limit",
                         PGUbarT(i)       "Power generation upper limit",
                         ThetaLbar(i)     "Power angle lower limit",
                         ThetaUbar(i)     "Power angle upper limit",
                         B(i,i)           "Admittance matrix",
                         PLoad(i)         "Load value",
                         PLbar(i,i)       "Line Flow Limits",
                         LineStat(i,i)    "Line Status",
                         NodeStat(i)      "Node Status",
                         GenStat(i)       "Generator Status",
                         Rampbar(i)       "Ramp rate limit",
                         IntDur           "Duration of a typical interval",
                         CritFrac(i,c)    "Fraction of loads critical and non-critical",
                         CritVal(c)       "Value of the critical load",
                         PGBegin(i)       "Generation at the begining";


$if not set gdxincname $abort 'no include file name for data file provided'
$gdxin %gdxincname%
$load PGLbarT, PGUbarT, ThetaLbar, ThetaUbar, B, PLoad, PLbar, LineStat, NodeStat, GenStat, Rampbar, IntDur, CritFrac, CritVal, PGBegin
$gdxin

binary variable   OutGen(i)            "Generator operating condition",
                  OutGen1(i)            "Generator operating condition";

variable          Theta(i)             "Power angle",
                  PGn(i)               "generation",
                  DPLoadC(i)           "Critical Load Not Served",
                  DPLoadNC(i)          "Non-Critical Load Not Served",
                  LineFlow(i,j)        "Line Flow",
                  z                      "The objective funcion value";

equations        Obj           "The equation representing the objective function",
                 GenFlow(i),
                 BusLineFlow1(i,j);


Obj..                    z =e= sum(i, CritVal('1')*DPLoadC(i) + CritVal('2')*DPLoadNC(i)) +  epsilon*sum(i,(PGn(i) - PGBegin(i))*(PGn(i) - PGBegin(i)));
GenFlow(i)..           PGn(i) - PLoad(i)*NodeStat(i) + DPLoadC(i) + DPLoadNC(i) - sum(j,LineFlow(i,j)) =e= 0;
BusLineFlow1(i,j)..    LineFlow(i,j) - (Theta(i) - Theta(j))*B(i,j)*LineStat(i,j)  =e= 0;
Theta.lo(i) = ThetaLbar(i);
Theta.up(i) = ThetaUbar(i);
DPLoadC.lo(i) = 0;
DPLoadC.up(i) = PLoad(i)*CritFrac(i,'1')*NodeStat(i);
DPLoadNC.lo(i) = 0;
DPLoadNC.up(i) = PLoad(i)*CritFrac(i,'2')*NodeStat(i);

equations        LineFlowLim1(i,j)           "Line Flow Status",
                 LineFlowLim2(i,j)           "Line Flow Status";

* Line Related Constraint
LineFlowLim1(i,j)..    LineFlow(i,j) =g= -PLbar(i,j)*LineStat(i,j);
LineFlowLim2(i,j)..    LineFlow(i,j) =l= PLbar(i,j)*LineStat(i,j);


equations        GenOpt1(i)                    "Generator operating condition",
                 GenOpt2(i)                    "Generator operating condition",
                 RRateLimit1(i)                "Generator output based on ramp rate",
                 RRateLimit2(i)                "Generator output based on ramp rate";

* Generator Related Constraint
GenOpt1(i)..    PGn(i) =g= PGLbarT(i)*NodeStat(i)*OutGen(i);
GenOpt2(i)..    PGn(i) =l= PGUbarT(i)*NodeStat(i)*OutGen(i);
RRateLimit1(i).. PGn(i) - PGBegin(i)  =l=  IntDur*Rampbar(i)*NodeStat(i)*OutGen(i);
* RRateLimit2(i).. PGn(i) - PGBegin(i)  =g= -IntDur*Rampbar(i)*NodeStat(i)*OutGen(i) - PGUbarT(i)*(1 - OutGen(i));
RRateLimit2(i).. PGn(i) - PGBegin(i) =g= -IntDur*Rampbar(i)*NodeStat(i)*OutGen(i) - 100*Rampbar(i)*(1 - OutGen(i));
OutGen.up(i) = GenStat(i);



model PART /all/;
* PART.nodLim = 500;
* PART.domlim  = 10000;
*PART.optfile = 1;
PART.optcr = 0.0001

option miqcp = scip;
solve PART minimizing z using miqcp

* execute_unload 'soln01', Theta;

parameter flow(i,j), flowP(i,j), flow2(i,j), ZVal, ZValP(i), PGn_total, Load_loss, DPLoad(i), PLoad_(i), p_solved_nc, p_solved_c, p_solved;
variable OutGen_val(i), DPLoad_val(i), PLoad_served(i);
flow(i,j)  = LineFlow.l(i,j) - (Theta.l(i) - Theta.l(j))*B(i,j) + (1 - LineStat(i,j))*M2;
flowP(i,j) = LineFlow.l(i,j) - (Theta.l(i) - Theta.l(j))*B(i,j) - (1 - LineStat(i,j))*M2;
flow2(i,j) =  flow(i,j) + flow(j,i);

ZVal    = sum(i, CritVal('1')*DPLoadC.l(i) + CritVal('2')*DPLoadNC.l(i));
ZValP(i) = CritVal('1')*DPLoadC.l(i) + CritVal('2')*DPLoadNC.l(i);
PGn_total = sum(i, PGn.l(i));
DPLoad(i) = DPLoadC.l(i) + DPLoadNC.l(i);
PLoad_(i) = PLoad(i) - DPLoad(i); 
Load_loss = sum(i, DPLoad(i));
p_solved_c = sum(i, PLoad(i)*CritFrac(i,'1')) -  sum(i, DPLoadC.l(i));
p_solved_nc = sum(i, PLoad(i)*CritFrac(i,'2')) - sum(i, DPLoadNC.l(i));
p_solved = sum(i, PLoad_(i));
OutGen_val.lo(i) = 0;
OutGen_val.up(i) = 0;
OutGen_val.l(i) = 0;
OutGen_val.fx(i) = OutGen.l(i);
DPLoad_val.l(i) = 0;
DPLoad_val.lo(i) = 0;
DPLoad_val.up(i) = 0;
DPLoad_val.fx(i) = DPLoad(i);
PLoad_served.l(i) = 0;
PLoad_served.lo(i) = 0;
PLoad_served.up(i) = 0;
PLoad_served.fx(i) = PLoad_(i);
parameter ModStat        "Model status";
ModStat = PART.modelstat;

display  ModStat, ZVal, ZValP, Pgn_total, DPLoad, PLoad_, Load_loss, Theta.l, LineFlow.l;

'''




