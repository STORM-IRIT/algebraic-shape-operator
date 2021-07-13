/*
 * David Coeurjolly <david.coeurjolly@liris.cnrs.fr>
 * Copyright (c) 2018 CNRS Université de Lyon
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the Halton project.
 */

#include <random>
#include <chrono>
#include <assert.h>
#include "Geogram/Delaunay_psm.h"

struct SamplerLloyd
{
  SamplerLloyd()
  {
    GEO::initialize();
    periodic_ = false;
  }
  
  ~SamplerLloyd()
  {
    if (!delaunay_) delete delaunay_;
  }
  
  template<typename RealPoint>
  void init(const std::vector<RealPoint>& pts, double minDomain, double maxDomain)
  {
    minDomain_ = minDomain;
    maxDomain_ = maxDomain;
    
    delaunay_ = new GEO::PeriodicDelaunay3d(false, 1.0);
    delaunay_->set_keeps_infinite(true);
    nb_points_ = pts.size();
    GEO::index_t nb_points = GEO::index_t(nb_points_);
    points_.resize(3*nb_points);
    new_points_.resize(3*nb_points);
    
    for(GEO::index_t i=0; i<nb_points; ++i)
    {
      points_[3*i]   = (pts[i][0] - minDomain_)/(maxDomain_ - minDomain_);
      points_[3*i+1] = (pts[i][1] - minDomain_)/(maxDomain_ - minDomain_);
      points_[3*i+2] = (pts[i][2] - minDomain_)/(maxDomain_ - minDomain_);
    }
    delaunay_->set_vertices(nb_points, points_.data());
    delaunay_->compute();
  }
  
  void Lloyd(unsigned int nbSteps)
  {
    for(auto i=0u; i < nbSteps; ++i)
    {
      std::cout<<"+"<<std::flush;
      Lloyd_step();
    }
    std::cout<<std::endl;
  }
  
  void Lloyd_step()
  {
    GEO::ConvexCell C;
    for(GEO::index_t v=0; v < nb_points_; ++v)
    {
      get_cell(v, C);
      GEO::vec3 g = C.barycenter();
      new_points_[3*v]   = g.x;
      new_points_[3*v+1] = g.y;
      new_points_[3*v+2] = g.z;
    }
    // In periodic mode, points may escape out of
    // the domain. Relocate them in [0,1]^3
    for(GEO::index_t i=0; i<new_points_.size(); ++i)
    {
      if(new_points_[i] < 0.)
        new_points_[i] += 1.0;
      
      if(new_points_[i] > 1.0)
        new_points_[i] -= 1;
    }
    points_.swap(new_points_);
    delaunay_->set_vertices(nb_points_, points_.data());
    delaunay_->compute();
  }
  
  template<typename RealPoint>
  void getSamples(std::vector<RealPoint> &samples)
  {
    assert(samples.size() == nb_points_);
    for(GEO::index_t i=0; i<nb_points_; ++i)
    samples[i] = RealPoint( points_[3*i]*(maxDomain_-minDomain_) + minDomain_ ,
                           points_[3*i+1]*(maxDomain_-minDomain_) + minDomain_,
                           points_[3*i+2]*(maxDomain_-minDomain_) + minDomain_);
  }
  
  
protected:
  double minDomain_;
  double maxDomain_;
  std::vector<double> points_;
  std::vector<double> new_points_;
  GEO::PeriodicDelaunay3d *delaunay_;
  bool periodic_;
  unsigned int nb_points_;
  GEO::PeriodicDelaunay3d::IncidentTetrahedra W_;
  
  
  void get_cell(GEO::index_t v, GEO::ConvexCell& C)
  {
    delaunay_->copy_Laguerre_cell_from_Delaunay(v, C, W_);
    if(!periodic_)
    {
      C.clip_by_plane(GEO::vec4( 1.0, 0.0, 0.0, 0.0));
      C.clip_by_plane(GEO::vec4(-1.0, 0.0, 0.0, 1.0));
      C.clip_by_plane(GEO::vec4( 0.0, 1.0, 0.0, 0.0));
      C.clip_by_plane(GEO::vec4( 0.0,-1.0, 0.0, 1.0));
      C.clip_by_plane(GEO::vec4( 0.0, 0.0, 1.0, 0.0));
      C.clip_by_plane(GEO::vec4( 0.0, 0.0,-1.0, 1.0));
    }
    C.compute_geometry();
  }
  
};
