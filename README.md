# Steering Geometry Solver using MATLAB

A MATLAB-based steering geometry solver developed to predict individual wheel steering angles, tyre slip angles, and tyre forces during steady-state cornering. The project combines vehicle dynamics, the Pacejka ’96 Magic Formula tyre model, and aerodynamic effects to analyse steering behaviour under realistic operating conditions.

> **Project Status:** ✅ Completed

---

## Project Overview

Conventional steering geometry calculations often rely on ideal Ackermann steering geometry and simplified bicycle models, which do not fully capture the tyre behaviour and vehicle dynamics involved in real-world cornering.

This project develops a more comprehensive approach to steering analysis by integrating tyre force prediction with four-wheel vehicle dynamics.

The solver:

- Implements the **Pacejka ’96 Magic Formula tyre model** for tyre force prediction.
- Models **steady-state cornering of a four-wheel vehicle**.
- Predicts individual wheel slip angles and tyre forces.
- Calculates inner and outer wheel steering angles based on vehicle cornering behaviour.
- Incorporates aerodynamic downforce and drag and evaluates their effects on vehicle dynamics.
- Goes beyond ideal 100% Ackermann assumptions and the conventional bicycle model to analyse steering geometry under realistic cornering conditions.

The completed solver serves as a MATLAB-based tool for analysing steering geometry, understanding tyre behaviour, and evaluating steering configurations to make better use of available tyre grip.

