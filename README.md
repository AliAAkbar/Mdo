# MDO Aircraft Optimization Project

## Goal
This project performs multidisciplinary optimization (MDO) of an aircraft configuration using OpenVSP and MATLAB.

## Disciplines
- Aerodynamics (CL, CD estimation)
- Geometry parameterization
- Optimization (Genetic Algorithm + Multi-objective)

## Objectives
1. Maximize CL/CD
2. Minimize weight
3. Satisfy aerodynamic constraints

## Methodology
The optimization loop follows:
1. Design variable generation
2. Geometry update (OpenVSP)
3. Aerodynamic evaluation
4. Objective function evaluation
5. Genetic algorithm update
6. Convergence check

## Key Output
- Pareto front
- Optimal geometry parameters
- Convergence history

## Execution
Run:
main.m
