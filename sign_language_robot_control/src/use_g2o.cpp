// g2o
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>

#include <g2o/solvers/cholmod/linear_solver_cholmod.h>


// Setup
const int arm_dim = 6;
const int hand_dim = 12;
const int vertex_dim = (arm_dim + hand_dim) * 2 + 1;
typedef g2o::BlockSolver< g2o::BlockSolverTraits<vertex_dim, 1> > MyBlock; // plus time difference
typedef Eigen::Matrix<double, vertex_dim, 1> VectorNd; // 'N' for fixed-size, 'd' for double.

// Define my own vertex(May be should define vertex independently for time difference and joint states???)
class MyVertex: public g2o::BaseVertex<vertex_dim, VectorNd>
{

  public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MyVertex(){}

  virtual void setToOriginImpl() // Reset(Set the internal state of the vertex to 0)
  {
    _estimate.setZero(); //fill(0); // fill VectorNd with all 0's
  }
    
  virtual void oplusImpl( const double* update ) // Update(Possibly, mapping a local variation in the Euclidean space to a variation on the manifold. Not needed in our case, just use oridinary update rule since the trajectory and variation are both in the Euclidean space.)
  {
    _estimate += VectorNd(update);
  }

  // Read or Write, leave empty
  virtual bool read( istream& in ) {}
  virtual bool write( ostream& out ) const {}

};


// Define my own edge(Unary for joint angle limits)
class MyUnaryEdge: public g2o::BaseUnaryEdge<1, double, MyVertex>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CurveFittingEdge( double x ): BaseUnaryEdge(), _x(x) {}
    // 计算曲线模型误差
    void computeError()
    {
        const CurveFittingVertex* v = static_cast<const CurveFittingVertex*> (_vertices[0]);
        const Eigen::Vector3d abc = v->estimate();
        _error(0,0) = _measurement - std::exp( abc(0,0)*_x*_x + abc(1,0)*_x + abc(2,0) ) ;
    }
    virtual bool read( istream& in ) {}
    virtual bool write( ostream& out ) const {}
public:
    double _x;  // x 值， y 值为 _measurement
};





int main(int argc, char **argv)
{



  // 1 - Choose linear equation solver (PCG/CSparse/Cholmod)
  MyBlock::LinearSolverType* linear_solver = new g2o::LinearSolverCholmod<MyBlock::PoseMatrixType>(); 

  // 2 - Choose block solver
  MyBlock* block_solver_ptr = new MyBlock(linear_solver);

  // 3 - Choose optimization algorithm(GN/LM/Powell's dogleg)
  g2o::OptimizationAlgorithmLevenberg* algorithm = new g2o::OptimizationAlgorithmLevenberg(block_solver_ptr);

  // 4 - Construct hyper-graph
  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(algorithm);
  optimizer.setVerbose(true);

  // 5 - Define vertex and edge, add to hyper-graph 
  // Vertex:
  MyVertex *v = new MyVertex();
  v->setEstimate(VectorNd::Identity());
  v->setId(0);
  optimizer.addVertex(v);
  v->setEstimate(VectorNd::Identity());
  v->setId(1);
  optimizer.addVertex(v);
  // Edge:
  edge->setId(i);
  edge->setVertex();
  edge->setMeasurement();
  edge->setInformation(); // weights?
  optimizer.addEdge(edge);

  // 6 - Set optimization parameters and start optimization
  optimizer.initializeOptimization();
  optimizer.optimize(100);


  return 0;

}
