
% James Butcher
% Warehouse 2.0
% 2-26-2021


% ------------------------------------------------------------------------------
% Unpack Input
% ------------------------------------------------------------------------------

object(O, V) :- init(object(O, V), value(_, _)).
value(A, V) :- init(object(_, _), value(A, V)).

state(object(O, V1), value(A, V2), 0) :- init(object(O, V1), value(A, V2)).




% ------------------------------------------------------------------------------
% Goals
% ------------------------------------------------------------------------------

% Fulfill all orders
:- not state(object(order, D), value(line, pair(I, 0)), m), init(object(order, D), value(line, pair(I, U))).


% ------------------------------------------------------------------------------
% Sort and Object Declaration
% ------------------------------------------------------------------------------

direction(0, 1; 0, -1; 1, 0; -1, 0).
boolean(true, false).

% ------------------------------------------------------------------------------
% Other Object Initialization
% ------------------------------------------------------------------------------

% Robots start not carrying a shelf
state(object(robot, R), value(carrying, false), 0) :- object(robot, R).

% Statement indicating if two objects are at the same location
same_location(object(O1, V1), object(O2, V2), T) :- state(object(O1, V1), value(at, pair(X, Y)), T), state(object(O2, V2), value(at, pair(X, Y)), T).

% Indicate immoveable object locations for easy location matching
is_highway(X, Y) :- init(object(highway, H), value(at, pair(X, Y))).
%is_pickingstation(N, X, Y) :- init(object(pickingStation, N), value(at, pair(X, Y))).

%product_on_shelf(I, S)
shelf_at_picking_station(S, N, T) :- same_location(object(shelf, S), object(pickingStation, N), T).

% ------------------------------------------------------------------------------
% State Constraints
% ------------------------------------------------------------------------------

% If a robot is carrying a shelf, then the shelf must be at the same location as the robot in the next time step if the robot moves
state(object(shelf, S), value(at, pair(X2, Y2)), T+1) :- state(object(robot, R), value(carrying, true), T),
                                                         state(object(robot, R), value(at, pair(X1, Y1)), T),
                                                         state(object(shelf, S), value(at, pair(X1, Y1)), T),
                                                         state(object(robot, R), value(at, pair(X2, Y2)), T+1).

% No two robots can be at the same location at the same time
:- state(object(robot, R1), value(at, pair(X, Y)), T), state(object(robot, R2), value(at, pair(X, Y)), T), R1 != R2.

% Two shelves cannot be in the same location at the same time
:- state(object(shelf, S1), value(at, pair(X, Y)), T), state(object(shelf, S2), value(at, pair(X, Y)), T), S1 != S2.

% A robot cannot move and pick up a shelf at the same time
:- occurs(object(robot, R), move(direction(DX, DY)), T), occurs(object(robot, R), pickup, T).

% A robot cannot move and put down a shelf at the same time
:- occurs(object(robot, R), move(direction(DX, DY)), T), occurs(object(robot, R), putdown, T).

% A robot cannot move and deliver at the same time
:- occurs(object(robot, R), move(direction(DX, DY)), T), occurs(object(robot, R), deliver(O, I, Q), T).

% A robot cannot move and put down a shelf at the same time
:- occurs(object(robot, R), putdown, T), occurs(object(robot, R), deliver(O, I, Q), T).


% ------------------------------------------------------------------------------
% Actions
% ------------------------------------------------------------------------------

% Move action
%------------
state(object(robot, R), value(at, pair(X + DX, Y + DY)), T+1) :- occurs(object(robot, R), move(direction(DX, DY)), T), state(object(robot, R), value(at, pair(X, Y)), T).
% Cannot move out of bounds, i.e, to a location not declared in the beginning
:- occurs(object(robot, R), move(direction(DX, DY)), T), state(object(robot, R), value(at, pair(X, Y)), T), not 1{value(at, pair(X + DX, Y + DY))}.
% Avoid collisions with other robots - also takes care of making actions serializable
:- state(object(robot, R1), value(at, pair(X1, Y)), T), state(object(robot, R2), value(at, pair(X2, Y)), T), (X1 - X2) == 1, occurs(object(robot, R1), move(direction(-1, 0)), T).
:- state(object(robot, R1), value(at, pair(X1, Y)), T), state(object(robot, R2), value(at, pair(X2, Y)), T), (X1 - X2) == -1, occurs(object(robot, R1), move(direction(1, 0)), T).
:- state(object(robot, R1), value(at, pair(X, Y1)), T), state(object(robot, R2), value(at, pair(X, Y2)), T), (Y1 - Y2) == 1, occurs(object(robot, R1), move(direction(0, -1)), T).
:- state(object(robot, R1), value(at, pair(X, Y1)), T), state(object(robot, R2), value(at, pair(X, Y2)), T), (Y1 - Y2) == -1, occurs(object(robot, R1), move(direction(0, 1)), T).

% Pickup action
%--------------
state(object(robot, R), value(carrying, true), T+1) :- occurs(object(robot, R), pickup, T).
% Must be a shelf at the robot's location
:- occurs(object(robot, R), pickup, T), not 1{same_location(object(robot, R), object(shelf, S), T) : object(shelf, S)}1.
% Cannot pick up a shelf if the robot is already carrying a shelf
:- occurs(object(robot, R), pickup, T), state(object(robot, R), value(carrying, true), T).

% Putdown action
%---------------
state(object(robot, R), value(carrying, false), T+1) :- occurs(object(robot, R), putdown, T).
% Cannot put down a shelf if the robot isn't carrying a shelf
:- occurs(object(robot, R), putdown, T), state(object(robot, R), value(carrying, false), T).
% Cannot put down a shelf on a highway
:- occurs(object(robot, R), putdown, T), state(object(robot, R), value(at, pair(X, Y)), T), is_highway(X, Y).

% Deliver action
%---------------
%state(object(product, I), value(on, pair(S, U - Q)), T+1) :- occurs(object(robot, R), deliver(O, I, Q), T), state(object(product, I), value(on, pair(S, U)), T).
%state(object(order, O), value(line, pair(I, U - Q)), T+1) :- occurs(object(robot, R), deliver(O, I, Q), T), state(object(order, O), value(line, pair(I, U)), T).
state(object(product, I), value(on, pair(S, U - Q)), T+1) :- occurs(object(robot, R), deliver(O, I, Q), T), state(object(product, I), value(on, pair(S, U)), T).
state(object(order, O), value(line, pair(I, U - Q)), T+1) :- occurs(object(robot, R), deliver(O, I, Q), T), state(object(order, O), value(line, pair(I, U)), T).



% Cannot deliver if shelf is not at same location as correct pickingStation
%:- occurs(object(robot, R), deliver(O, I, Q), T), state(object(product, I), value(on, pair(S, U)), T), state(object(order, O), value(pickingStation, N), T),
%not same_location(object(robot, R), object(pickingStation, N), T).
 :- occurs(object(robot, R), deliver(O, I, Q), T), state(object(order, O), value(pickingStation, N), T), not same_location(object(robot, R), object(pickingStation, N), T).

:- occurs(object(robot, R), deliver(O, I, Q), T), state(object(robot, R), value(carrying, false), T).

:- occurs(object(robot, R), deliver(O, I, Q), T), state(object(product, I), value(on, pair(S, U)), T), state(object(order, O), value(pickingStation, N), T),
same_location(object, robot, R), object(shelf, S),
not 1{same_location(object(shelf, S), object(pickingStation, N), T)}.

%:- occurs(object(robot, R), deliver(O, I, Q), T), state(object(product, I), value(on, pair(S, U)), T), state(object(order, O), value(pickingStation, N), T),
%not shelf_at_picking_station(S, N, T).


% ------------------------------------------------------------------------------
% Domain-Independent Axioms
% ------------------------------------------------------------------------------

% Fluents are initially exogenous
%1{state(object(O, V), value(A, V2), 0) : value(A, V2)}1 :- object(O, V).
%1{state(object(node, N), value(at, pair(X, Y)), 0) : pair(X, Y)}1 :- object(node, N).
%1{state(object(robot, R), value(carrying, BB), 0) : boolean(BB)}1 :- object(robot, R).
%1{state(object(robot, R), value(at, pair(X, Y)), 0) : value(at, pair(X, Y))}1 :- object(robot, R).

% Actions are exogenous
{occurs(object(robot, R), move(direction(DX, DY)), T) : direction(DX, DY)} :- object(robot, R), T = 0..m-1.
{occurs(object(robot, R), pickup, T)} :- object(robot, R), T = 0..m-1.
{occurs(object(robot, R), putdown, T)} :- object(robot, R), T = 0..m-1.
{occurs(object(robot, R), deliver(O, I, U), T) : object(order, O), object(product, I), U = 1..4} :- object(robot, R), T = 0..m-1.

%{deliver(R,OI,with(SI,PR,DQ),T):orderAt(OI,object(node,ND),contains(PR,OQ),T), productOn(PR,object(shelf,SI),with(quantity,PQ),T), DQ=1..PQ}1:- R=1..NR, numRobots(NR), T=0..TN,TN=n-1.

% Existence and uniqueness of fluent values
%:- not 1{state(object(O, V), value(A, V2), 0) : value(A, V2)}1, object(O, V), T = 1..m.
%:- not 1{state(object(node, N), value(at, pair(X, Y)), T)}1, object(node, N), T = 1..m.
:- not 1{state(object(robot, R), value(carrying, B), T)}1, object(robot, R), T = 1..m.
:- not 1{state(object(robot, R), value(at, pair(X, Y)), T)}1, object(robot, R), T = 1..m.
:- not 1{state(object(shelf, S), value(at, pair(X, Y)), T)}1, object(shelf, S), T = 1..m.
:- not 1{state(object(pickingStation, N), value(at, pair(X, Y)), T)}1, object(pickingStation, N), T = 1..m.
:- not 1{state(object(product, I), value(on, pair(S, U)), T)}1, object(product, I), object(shelf, S), T = 1..m.
:- not 1{state(object(order, O), value(line, pair(I, U)), T)}1, object(order, O), T = 1..m.
:- not 1{state(object(order, O), value(pickingStation, N), T)}1, object(order, O), T = 1..m.

% Common sense law of inertia
%{state(object(O, V), value(A, V2), T+1)} :- state(object(O, V), value(A, V2), T), T = 0..m-1.
%{state(object(node, N), value(at, pair(X, Y)), T+1)} :- state(object(node, N), value(at, pair(X, Y)), T), T = 0..m-1.
{state(object(robot, R), value(carrying, B), T+1)} :- state(object(robot, R), value(carrying, B), T), T = 0..m-1. % Carrying shelf or not
{state(object(robot, R), value(at, pair(X, Y)), T+1)} :- state(object(robot, R), value(at, pair(X, Y)), T), T = 0..m-1.
{state(object(shelf, S), value(at, pair(X, Y)), T+1)} :- state(object(shelf, S), value(at, pair(X, Y)), T), T = 0..m-1.
{state(object(pickingStation, N), value(at, pair(X, Y)), T+1)} :- state(object(pickingStation, N), value(at, pair(X, Y)), T), T = 0..m-1.
{state(object(product, I), value(on, pair(S, U)), T+1)} :- state(object(product, I), value(on, pair(S, U)), T), T = 0..m-1.
{state(object(order, O), value(line, pair(I, U)), T+1)} :- state(object(order, O), value(line, pair(I, U)), T), T = 0..m-1.
{state(object(order, O), value(pickingStation, N), T+1)} :- state(object(order, O), value(pickingStation, N), T), T = 0..m-1.



%#show state/3.
#show occurs/3.

%
