#ifndef MPC_HPP
#define MPC_HPP

#include "ros/ros.h"
#include <eigen3/Eigen/Dense>
#include <iostream>
#include "OsqpEigen/OsqpEigen.h"

#define STATESIZE (3)
#define CONTROLSIZE (2)
#define MPCWINDOW (30)
class Mpc
{
private:
    //preview window
    //int mpc_window_;
    double dt_;
    double vr_;
    double wr_;
    //dynamics matrices
    Eigen::Matrix<double,STATESIZE,STATESIZE> a_;
    Eigen::Matrix<double,STATESIZE,CONTROLSIZE> b_;

    //constraints vector
    Eigen::Matrix<double,STATESIZE,1> x_max_, x_min_;
    Eigen::Matrix<double,CONTROLSIZE,1> u_max_, u_min_;

    //weight matrices
    Eigen::DiagonalMatrix<double, STATESIZE> Q_;
    Eigen::DiagonalMatrix<double, CONTROLSIZE> R_;

    //the initial and the reference state
    Eigen::Matrix<double,STATESIZE,1> x0_, x_ref_,state_;

    //QP problem matrices and vectors
    Eigen::SparseMatrix<double> hessian_;
    Eigen::VectorXd gradient_;
    Eigen::SparseMatrix<double> linear_matrix_;
    Eigen::VectorXd lower_bound_, upper_bound_; 

    // QP solution
    Eigen::VectorXd QP_solution_;

    OsqpEigen::Solver solver;


public:
    Mpc();
    ~Mpc();


    void setDynamicsMatrices(Eigen::Matrix<double,STATESIZE,STATESIZE> &a, Eigen::Matrix<double,STATESIZE,CONTROLSIZE> &b,double yaw_ref);
    void setInequalityConstraints(Eigen::Matrix<double,STATESIZE,1> &x_max, Eigen::Matrix<double,STATESIZE,1> &x_min,
                                    Eigen::Matrix<double,CONTROLSIZE,1> &u_max, Eigen::Matrix<double,CONTROLSIZE,1> &u_min);
    void setWeightMatrices(Eigen::DiagonalMatrix<double, STATESIZE> &Q, Eigen::DiagonalMatrix<double, CONTROLSIZE> R);
    void castMPCToQPHessian(const Eigen::DiagonalMatrix<double, STATESIZE> &Q, const Eigen::DiagonalMatrix<double, CONTROLSIZE> &R, int mpcWindow,
                        Eigen::SparseMatrix<double> &hessianMatrix);
    void castMPCToQPGradient(const Eigen::DiagonalMatrix<double, STATESIZE> &Q, const Eigen::Matrix<double,STATESIZE,1> &x_ref,
                            int mpc_window, Eigen::VectorXd &gradient);
    
    void castMPCToQPGradientVarXref(const Eigen::DiagonalMatrix<double, STATESIZE> &Q, const Eigen::Matrix3Xd &x_ref,
                            int mpc_window, Eigen::VectorXd &gradient);
    void castMPCToQPConstraintMatrix(const Eigen::Matrix<double,STATESIZE,STATESIZE> &dynamic_atrix, const Eigen::Matrix<double,STATESIZE,CONTROLSIZE> &control_matrix,
                                    int mpc_window, Eigen::SparseMatrix<double> &constraint_matrix);
    void castMPCToQPConstraintMatrix2(int mpcWindow, Eigen::SparseMatrix<double> &constraintMatrix);
    void castMPCToQPConstraintVectors(const Eigen::Matrix<double,STATESIZE,1> &x_max, const Eigen::Matrix<double,STATESIZE,1> &x_min,
                                        const Eigen::Matrix<double,CONTROLSIZE,1> &u_max, const Eigen::Matrix<double,CONTROLSIZE,1> &u_min,
                                        const Eigen::Matrix<double,STATESIZE,1> &x0, int mpc_window,
                                        Eigen::VectorXd &lower_bound, Eigen::VectorXd &upper_bound);
    void updateConstraintVectors();
    void updateConstraintMatrix();
    void solveMpc(Eigen::Matrix<double,STATESIZE,1> x0, Eigen::Matrix3Xd &x_ref,Eigen::Matrix<double,STATESIZE,1> current_state);
    Eigen::Matrix<double,CONTROLSIZE,1> getControlCmd();
    Eigen::VectorXd getPredictState();

    void testfunc(int);
    
};

#endif