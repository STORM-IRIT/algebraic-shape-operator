#include <iostream>
#include <random>
#include <vector>
#include <array>
#include <fstream>
#include <DGtal/base/Common.h>
#include <DGtal/helpers/StdDefs.h>
#include <DGtal/helpers/Shortcuts.h>
#include <DGtal/helpers/ShortcutsGeometry.h>

#include <Geogram/SamplerLloyd.hpp>
#include <CLI11.hpp>



// DGtal
using namespace DGtal;
using namespace Z3i;
typedef Shortcuts<Z3i::KSpace>         SH3;
typedef ShortcutsGeometry<Z3i::KSpace> SHG3;

//The implicit shape
CountedPtr<SH3::ImplicitShape3D> implicitShape;

//Main naive pcloud container
std::vector<SH3::RealPoint> pc;

//Quantities
std::vector<double> exactGauss;
std::vector<double> exactMean;
std::vector<RealPoint> projPoints;
std::vector<double> exactK1;
std::vector<double> exactK2;
std::vector<RealPoint> exactDir1;
std::vector<RealPoint> exactDir2;
std::vector<RealPoint> exactN;

// hard-coded bounds used for random sampling
void get_bounds(const std::string& surface, double& min, double& max)
{
    min = -15.5;
    max = +15.5;
    if(surface == "sphere1"   ){min= -2;   max= +2;   return;}
    if(surface == "sphere9"   ){min=-10;   max=+10;   return;}
    if(surface == "ellipsoid" ){min=-10;   max=+10;   return;}
    if(surface == "cylinder"  ){min=-15.5; max=+15.5; return;}
    if(surface == "torus"     ){min=-10;   max=+10;   return;}
    if(surface == "rcube"     ){min=-15.5; max=+15.5; return;}
    if(surface == "goursat"   ){min=-15.5; max=+15.5; return;}
    if(surface == "goursat-ho"){min=-15.5; max=+15.5; return;}
    if(surface == "distel"    ){min=-15.5; max=+15.5; return;}
    if(surface == "leopold"   ){min=-10;   max=+10;   return;}
    if(surface == "diabolo"   ){min=-15.5; max=+15.5; return;}
    if(surface == "heart"     ){min= -2;   max= +2;   return;}
    if(surface == "crixxi"    ){min= -2;   max= +2;   return;}
    std::cout << "Warning: unkown surface type, bounds set to dedault (-15.5,+15.5)" << std::endl;
}

//ImGUI handler
void samplingAndGenerate(const std::string& surface, unsigned int nbPts, unsigned int nbSteps, double epsilonRejection)
{
  auto params = SH3::defaultParameters() | SHG3::defaultParameters() |  SHG3::parametersGeometryEstimation();
  params( "polynomial", surface);
  implicitShape  = SH3::makeImplicitShape3D  ( params );
  // regardless of the surface, (mind,maxd) was equal to (-15.5,+15.5)
  // which slow down the sampling (for the unit sphere for instance)
//  auto digitized_shape = SH3::makeDigitizedImplicitShape3D( implicitShape, params );
//  double mind = digitized_shape->getDomain().lowerBound().min() - epsilonRejection;
//  double maxd = digitized_shape->getDomain().upperBound().max() + epsilonRejection;
  double mind, maxd;
  get_bounds(surface, mind, maxd);
  std::cout<<"Extent = "<<mind<<" "<<maxd<<std::endl;
  
  std::default_random_engine generator;
  std::uniform_real_distribution<double> distribution(mind,maxd);

  pc.clear();

  std::cout<<"Generating samples by rejection..."<<std::endl;
  //Rejection sampling close to the boundary
  unsigned int cpt=0;
  while (cpt < nbPts)
  {
    RealPoint p ({distribution(generator),distribution(generator),distribution(generator) });
    auto q = implicitShape->nearestPoint(p);
    if ((p-q).norm()  < epsilonRejection)
    {
      pc.push_back(p);
      ++cpt;
    }
  }
  std::cout<<nbPts<<" samples."<<std::endl;
  //Optimization
  std::cout<<"Optimizing..."<<std::endl;
  SamplerLloyd optimizer;
  cpt=0u;
  std::vector<RealPoint> bary(nbPts);
  while (cpt<nbSteps)
  {
    std::cout<<"+"<<std::flush;
    optimizer.init(pc,mind,maxd);
    optimizer.Lloyd_step();
    optimizer.getSamples(bary);
    for(auto i=0u; i < pc.size(); ++i)
    {
      auto q = implicitShape->nearestPoint(pc[i]);
      pc[i] += 0.8*(q-pc[i]) + 0.2*(bary[i]-pc[i]); //weighting the gradients
    }
    ++cpt;
  }
  //Last step is a projection one.
  for(auto i=0u; i < pc.size(); ++i)
  {
    auto q = implicitShape->nearestPoint(pc[i]);
    pc[i] = q;
  }
  std::cout<<std::endl;
  std::cout<<"done "<<pc.size()<<std::endl;
  
  //Reproject
  projPoints.clear();
  exactMean.clear();
  exactGauss.clear();
  exactK1.clear();
  exactK2.clear();
  exactN.clear();
  exactDir1.clear();
  exactDir2.clear();
  
  for(auto p: pc)
  {
    exactMean.push_back(implicitShape->meanCurvature(p));
    exactGauss.push_back(implicitShape->gaussianCurvature(p));
    double k1,k2;
    implicitShape->principalCurvatures(p, k1, k2);
    exactK1.push_back(k1);
    exactK2.push_back(k2);
    RealPoint d1,d2;
    implicitShape->principalDirections(p, d1, d2);
    exactDir1.push_back(d1);
    exactDir2.push_back(d2);
    exactN.push_back( implicitShape->gradient(p).getNormalized() );
  }
}



int main(int argc, char **argv)
{
  CLI::App app{"Sampling implicit polynomial surface"};
  
  unsigned int nbPts = 10;
  app.add_option("-n,--nbPts", nbPts, "Number of samples");
  double epsilon=.5;
  app.add_option("-e,--epsilon", epsilon, "Epsilon for the rejection sampling");
  double nbSteps=40;
  app.add_option("--nbSteps", nbSteps, "Number of steps of the Lloyd relaxation");
  bool XYZOnly=false;
  app.add_flag("--XYZOnly", XYZOnly, "Export point coordinates only (default: false)");
  std::string outputFilename;
  app.add_option("-o,--output", outputFilename, "Output filename")->required();
  std::string surface = "goursat";
  app.add_option("-s,--surface", surface, "Surface [goursat|...]");
  
  CLI11_PARSE(app, argc, argv);
  
  //Main loop
  samplingAndGenerate(surface,nbPts,nbSteps,epsilon);
  
  std::ofstream ofs (outputFilename, std::ofstream::out);
  ofs << "# x y x Gauss_Curvature Mean_Curvature nx ny nz k1 k2 d1x d1y d1z d2x d2y d2z\n";
  for(auto i = 0u; i < nbPts; ++i)
  {
    ofs << pc[i][0] << " "<<  pc[i][1] << " "<<  pc[i][2];
    if (!XYZOnly)
    {
      ofs << " "<<  exactGauss[i] << " "<<  exactMean[i]<< " " ;
      ofs << exactN[i][0] << " "<<  exactN[i][1]<< " " <<  exactN[i][2] << " ";
      ofs << exactK1[i] << " "<<exactK2[i]<<" ";
      ofs << exactDir1[i][0]<< " " <<  exactDir1[i][1] << " "<<  exactDir1[i][2]<<" ";
      ofs << exactDir2[i][0] << " "<<  exactDir2[i][1] << " "<<  exactDir2[i][2];
    }
    ofs <<std::endl;
  }
  ofs.close();
  
  
  return 0;
}
