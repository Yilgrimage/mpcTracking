#include "task2_pkg/mpc.hpp"

Mpc::Mpc() {
    dt_ = 0.1;
    vr_ = 0.1;
    wr_ = 0;
    x0_ = Eigen::Matrix<double,STATESIZE,1>::Zero();
    state_ = Eigen::Matrix<double,STATESIZE,1>::Zero();

    x_ref_ << 0, 0, 0;
    setDynamicsMatrices(a_, b_,0);
    setInequalityConstraints(x_max_, x_min_, u_max_, u_min_);
    setWeightMatrices(Q_, R_);

    //cast the MPC problem as QP problem
    castMPCToQPHessian(Q_, R_, MPCWINDOW, hessian_);
    castMPCToQPGradient(Q_, x_ref_, MPCWINDOW, gradient_);
    castMPCToQPConstraintMatrix(a_, b_, MPCWINDOW, linear_matrix_);
    castMPCToQPConstraintVectors(x_max_, x_min_, u_max_, u_min_, x0_, MPCWINDOW, lower_bound_, upper_bound_);
    
    //instantiate the solver
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);

    solver.data()->setNumberOfVariables(STATESIZE * (MPCWINDOW + 1) + CONTROLSIZE * MPCWINDOW);
    solver.data()->setNumberOfConstraints(2 * STATESIZE * (MPCWINDOW + 1) + CONTROLSIZE * MPCWINDOW);
    if (!solver.data()->setHessianMatrix(hessian_)) return;
    if (!solver.data()->setGradient(gradient_)) return;
    if (!solver.data()->setLinearConstraintsMatrix(linear_matrix_)) return;
    if (!solver.data()->setLowerBound(lower_bound_)) return;
    if (!solver.data()->setUpperBound(upper_bound_)) return;
    if (!solver.initSolver()) return;

}
Mpc::~Mpc() {}

void Mpc::solveMpc(Eigen::Matrix<double,STATESIZE,1> x0, Eigen::Matrix3Xd &x_ref,Eigen::Matrix<double,STATESIZE,1> current_state) {
    
    //update solver
    x0_ = x0;
    state_ = current_state;
    //constraint vectors -> x0
    updateConstraintVectors();
    //constraint matrix -> a_, b_,so update solver.linear_matrix_
    updateConstraintMatrix();
    //Gradient -> x_ref_, so update solver.gradient_
    castMPCToQPGradientVarXref(Q_, x_ref, MPCWINDOW, gradient_);
    solver.updateGradient(gradient_);
    //controller input and QPSolution vector

    // solve the QP problem
    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return;

    // get QP solution
    QP_solution_ = solver.getSolution();
    //Eigen::Matrix<double,CONTROLSIZE,1> output = QP_solution_.block(STATESIZE * (MPCWINDOW + 1), 0, CONTROLSIZE, 1);
    //vr_ += output(0);
    //wr_ = output(1);
    //cout << "QP_solution_ :" << QP_solution_.transpose()  << endl; 
    return;
}

Eigen::Matrix<double,CONTROLSIZE,1> Mpc::getControlCmd() {
    Eigen::Matrix<double,CONTROLSIZE,1> cmd = QP_solution_.block(STATESIZE * (MPCWINDOW + 1), 0, CONTROLSIZE, 1);
    cmd(0) =  cmd(0) + vr_;
    cmd(1) =  cmd(1) + wr_;
    return cmd;
}

Eigen::VectorXd Mpc::getPredictState() {
    Eigen::VectorXd predict_states;
    predict_states = QP_solution_.block(0,0,STATESIZE*MPCWINDOW,1);
    return predict_states;
}

void Mpc::setDynamicsMatrices(Eigen::Matrix<double,STATESIZE,STATESIZE> &a, Eigen::Matrix<double,STATESIZE,CONTROLSIZE> &b,double yaw) {
    double tmp_sin = sin(yaw);
    double tmp_cos = cos(yaw);
    a << 1,0,-dt_*vr_*tmp_sin, 
        0,1,dt_*vr_*tmp_cos, 
        0,0,1;
    b << dt_*tmp_cos, 0,
        dt_*tmp_sin, 0,
        0, dt_;
}

void Mpc::setInequalityConstraints(Eigen::Matrix<double,STATESIZE,1> &x_max, Eigen::Matrix<double,STATESIZE,1> &x_min,
                                    Eigen::Matrix<double,CONTROLSIZE,1> &u_max, Eigen::Matrix<double,CONTROLSIZE,1> &u_min) {
    // input inequality constraints
    u_min << -0.1-vr_, -2.0-wr_;
    u_max << 0.2-vr_, 2.0-wr_;

    // state inequality constraints  
    x_min << -OsqpEigen::INFTY, -OsqpEigen::INFTY, -OsqpEigen::INFTY;
    x_max << OsqpEigen::INFTY, OsqpEigen::INFTY, OsqpEigen::INFTY;                                
}

void Mpc::setWeightMatrices(Eigen::DiagonalMatrix<double, STATESIZE> &Q, Eigen::DiagonalMatrix<double, CONTROLSIZE> R) {
    Q.diagonal() << 10 ,10, 0.0;
    R.diagonal() << 0.3, 0;
}


void Mpc::castMPCToQPHessian(const Eigen::DiagonalMatrix<double, STATESIZE> &Q, const Eigen::DiagonalMatrix<double, CONTROLSIZE> &R, int mpcWindow,
                        Eigen::SparseMatrix<double> &hessianMatrix)
{

    hessianMatrix.resize(STATESIZE*(mpcWindow+1) + CONTROLSIZE * mpcWindow, STATESIZE*(mpcWindow+1) + CONTROLSIZE * mpcWindow);

    //populate hessian matrix
    for(int i = 0; i<STATESIZE*(mpcWindow+1) + CONTROLSIZE * mpcWindow; i++){
        if(i < STATESIZE*(mpcWindow+1)){
            int posQ=i%STATESIZE;
            float value = Q.diagonal()[posQ];
            if(value != 0)
                hessianMatrix.insert(i,i) = value;
        }
        else{
            int posR=i%CONTROLSIZE;
            float value = R.diagonal()[posR];
            if(value != 0)
                hessianMatrix.insert(i,i) = value;
        }
    }
}

void Mpc::castMPCToQPGradient(const Eigen::DiagonalMatrix<double, STATESIZE> &Q, const Eigen::Matrix<double, STATESIZE, 1> &xRef, int mpcWindow,
                         Eigen::VectorXd &gradient)
{

    Eigen::Matrix<double,STATESIZE,1> Qx_ref;
    Qx_ref = Q * (-xRef);

    // populate the gradient vector
    gradient = Eigen::VectorXd::Zero(STATESIZE*(mpcWindow+1) +  CONTROLSIZE*mpcWindow, 1);
    for(int i = 0; i<STATESIZE*(mpcWindow+1); i++){
        int posQ=i%STATESIZE;
        float value = Qx_ref(posQ,0);
        gradient(i,0) = value;
    }
}

void Mpc::castMPCToQPGradientVarXref(const Eigen::DiagonalMatrix<double, STATESIZE> &Q, const Eigen::Matrix3Xd &x_ref,
                            int mpc_window, Eigen::VectorXd &gradient) {
    // populate the gradient vector
    gradient = Eigen::VectorXd::Zero(STATESIZE * (mpc_window + 1) + CONTROLSIZE * mpc_window, 1);

    for (int i = 0; i < mpc_window + 1; ++i) {
        Eigen::Vector3d Qx_ref = Q * (-x_ref.col(i));
        gradient.block(STATESIZE * i, 0, STATESIZE, 1) = Qx_ref;
    }                            
}


void Mpc::castMPCToQPConstraintMatrix(const Eigen::Matrix<double, STATESIZE, STATESIZE> &dynamicMatrix, const Eigen::Matrix<double, STATESIZE, CONTROLSIZE> &controlMatrix,
                                 int mpcWindow, Eigen::SparseMatrix<double> &constraintMatrix)
{
    constraintMatrix.resize(STATESIZE*(mpcWindow+1)  + STATESIZE*(mpcWindow+1) + CONTROLSIZE * mpcWindow, STATESIZE*(mpcWindow+1) + CONTROLSIZE * mpcWindow);

    // populate linear constraint matrix
    for(int i = 0; i<STATESIZE*(mpcWindow+1); i++){
        constraintMatrix.insert(i,i) = -1;
    }

    for(int i = 0; i < mpcWindow; i++)
        for(int j = 0; j<STATESIZE; j++)
            for(int k = 0; k<STATESIZE; k++){
                float value = dynamicMatrix(j,k);
                if(value != 0){
                    constraintMatrix.insert(STATESIZE * (i+1) + j, STATESIZE * i + k) = value;
                }
            }
 
    for(int i = 0; i < mpcWindow; i++)
        for(int j = 0; j < STATESIZE; j++)
            for(int k = 0; k < CONTROLSIZE; k++){
                float value = controlMatrix(j,k);
                if(value != 0){
                    constraintMatrix.insert(STATESIZE*(i+1)+j, CONTROLSIZE*i+k+STATESIZE*(mpcWindow + 1)) = value;
                }
            }

    for(int i = 0; i<STATESIZE*(mpcWindow+1) + CONTROLSIZE*mpcWindow; i++){
        constraintMatrix.insert(i+(mpcWindow+1)*STATESIZE,i) = 1;
    }
}

void Mpc::castMPCToQPConstraintMatrix2(int mpcWindow, Eigen::SparseMatrix<double> &constraintMatrix)
{
    constraintMatrix.resize(STATESIZE*(mpcWindow+1)  + STATESIZE*(mpcWindow+1) + CONTROLSIZE * mpcWindow, STATESIZE*(mpcWindow+1) + CONTROLSIZE * mpcWindow);

    // populate linear constraint matrix
    for(int i = 0; i<STATESIZE*(mpcWindow+1); i++){
        constraintMatrix.insert(i,i) = -1;
    }

    double tmp_yaw = state_(2);
    for(int i = 0; i < mpcWindow; i++)
    {
        //update dynamic matrix
        setDynamicsMatrices(a_,b_,tmp_yaw);
        
        for(int j = 0; j<STATESIZE; j++)
            for(int k = 0; k<STATESIZE; k++){
                float value = a_(j,k);
                if(value != 0){
                    constraintMatrix.insert(STATESIZE * (i+1) + j, STATESIZE * i + k) = value;
                }
            }
        for(int j = 0; j < STATESIZE; j++)
            for(int k = 0; k < CONTROLSIZE; k++){
                float value = b_(j,k);
                if(value != 0){
                    constraintMatrix.insert(STATESIZE*(i+1)+j, CONTROLSIZE*i+k+STATESIZE*(mpcWindow + 1)) = value;
                }
            }
        if(i < mpcWindow-1)
        tmp_yaw = tmp_yaw + (QP_solution_(CONTROLSIZE*(i+1)+1+STATESIZE*(mpcWindow+1))+ wr_)*dt_;
        else
        tmp_yaw = tmp_yaw + (QP_solution_(CONTROLSIZE*(i)+1+STATESIZE*(mpcWindow+1))+ wr_)*dt_;

    }

    for(int i = 0; i<STATESIZE*(mpcWindow+1) + CONTROLSIZE*mpcWindow; i++){
        constraintMatrix.insert(i+(mpcWindow+1)*STATESIZE,i) = 1;
    }
}

void Mpc::castMPCToQPConstraintVectors(const Eigen::Matrix<double, STATESIZE, 1> &xMax, const Eigen::Matrix<double, STATESIZE, 1> &xMin,
                                   const Eigen::Matrix<double, CONTROLSIZE, 1> &uMax, const Eigen::Matrix<double, CONTROLSIZE, 1> &uMin,
                                   const Eigen::Matrix<double, STATESIZE, 1> &x0,
                                   int mpcWindow, Eigen::VectorXd &lowerBound, Eigen::VectorXd &upperBound)
{
    // evaluate the lower and the upper inequality vectors
    Eigen::VectorXd lowerInequality = Eigen::MatrixXd::Zero(STATESIZE*(mpcWindow+1) +  CONTROLSIZE * mpcWindow, 1);
    Eigen::VectorXd upperInequality = Eigen::MatrixXd::Zero(STATESIZE*(mpcWindow+1) +  CONTROLSIZE * mpcWindow, 1);
    for(int i=0; i<mpcWindow+1; i++){
        lowerInequality.block(STATESIZE*i,0,STATESIZE,1) = xMin;
        upperInequality.block(STATESIZE*i,0,STATESIZE,1) = xMax;
    }
    for(int i=0; i<mpcWindow; i++){
        lowerInequality.block(CONTROLSIZE * i + STATESIZE * (mpcWindow + 1), 0, CONTROLSIZE, 1) = uMin;
        upperInequality.block(CONTROLSIZE * i + STATESIZE * (mpcWindow + 1), 0, CONTROLSIZE, 1) = uMax;
    }

    // evaluate the lower and the upper equality vectors
    Eigen::VectorXd lowerEquality = Eigen::MatrixXd::Zero(STATESIZE*(mpcWindow+1),1 );
    Eigen::VectorXd upperEquality;
    lowerEquality.block(0,0,STATESIZE,1) = -x0;
    upperEquality = lowerEquality;
    lowerEquality = lowerEquality;

    // merge inequality and equality vectors
    lowerBound = Eigen::MatrixXd::Zero(2*STATESIZE*(mpcWindow+1) +  CONTROLSIZE*mpcWindow,1 );
    lowerBound << lowerEquality,
        lowerInequality;

    upperBound = Eigen::MatrixXd::Zero(2*STATESIZE*(mpcWindow+1) +  CONTROLSIZE*mpcWindow,1 );
    upperBound << upperEquality,
        upperInequality;
}


void Mpc::updateConstraintVectors()
{
    lower_bound_.block(0,0,STATESIZE,1) = -x0_;
    upper_bound_.block(0,0,STATESIZE,1) = -x0_;
    solver.updateBounds(lower_bound_, upper_bound_);
}

void Mpc::updateConstraintMatrix()
{
    static bool first_time = true;
    if (first_time)
    {
        setDynamicsMatrices(a_, b_, state_(2));
        castMPCToQPConstraintMatrix(a_, b_, MPCWINDOW, linear_matrix_);
        first_time = false;
    }
    else
    {
    castMPCToQPConstraintMatrix2(MPCWINDOW, linear_matrix_);
    }
    solver.updateLinearConstraintsMatrix(linear_matrix_);

    //     setDynamicsMatrices(a_, b_, state_(2));
    //     castMPCToQPConstraintMatrix(a_, b_, MPCWINDOW, linear_matrix_);
    // solver.updateLinearConstraintsMatrix(linear_matrix_);


}

void Mpc::testfunc(int i)
{
    std::cout << "testfunc" << std::endl;
    return;

}
