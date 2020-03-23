/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;
using std::default_random_engine;
using std::normal_distribution;
using std::uniform_real_distribution;
using std::max_element;

using std::cout;
using std::endl;

void ParticleFilter::init(double x, double y, double theta, double std[]) {

  // Arbitrary, static number of particles in particle filter
  num_particles = 50;

  // Create normal (Gaussian) distributions for x, y, and theta
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);

  // Generate random particles around the initial estimate provided
  default_random_engine gen;

  for (int i = 0; i < num_particles; i++) {
  	Particle p;								// init new particle

    p.id = i;
    p.x = dist_x(gen);
    p.y = dist_y(gen);
    p.theta = dist_theta(gen);
    p.weight = 1;

    particles.push_back(p);		// append particle object to end of list

    // Diagnostic output
    // cout << "Init: " << p.id << ": " << p.x << ", " << p.y << ", " << p.theta
    // 		 << endl;

  }

  // Initialize vector of weights to all 1's
  weights.resize(num_particles, 1);

  is_initialized = true;	// init all done

  // Diagnostic outputs
  // cout << endl << "Size of particles vector: " << particles.size() << endl;
  // cout << "Size of weights vector: " << weights.size() << endl;
  // cout << "is_initialized = " << is_initialized << endl;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {

  // Create normal (Gaussian) distributions for x, y, and theta
	// These center around 0 instead of read-in values, to avoid unnecessary
	// recalculation when processing each particle
  normal_distribution<double> dist_x(0, std_pos[0]);
  normal_distribution<double> dist_y(0, std_pos[1]);
  normal_distribution<double> dist_theta(0, std_pos[2]);

  // Generate random Gaussian noise to add to predictions
  default_random_engine gen;
  for (unsigned int i = 0; i < particles.size(); i++) {
    double x_noise = dist_x(gen);
    double y_noise = dist_y(gen);
    double theta_noise = dist_theta(gen);
    double v_dt = velocity/yaw_rate;
    double old_dir = particles[i].theta;
    double dir_change = old_dir + yaw_rate * delta_t;

    // Diagnostic message
    // cout << "Predicting for particle " << i << ": Noise {" << x_noise << ", "
    // 		 << y_noise << ", " << theta_noise << "}, Old X/Y/T {"
    // 		 << particles[i].x << ", " << particles[i].y << ", "
    // 		 << particles[i].theta << "}, ";

    // Handle divide-by-zero case if yaw rate is zero
    if (yaw_rate) {
      particles[i].x += v_dt * (sin(dir_change)-sin(old_dir)) + x_noise;
      particles[i].y += v_dt * (cos(old_dir)-cos(dir_change)) + y_noise;
      particles[i].theta = dir_change + theta_noise;
    } else {
      particles[i].x += velocity * delta_t + x_noise;
      particles[i].y += y_noise;
      particles[i].theta += theta_noise;
    }

    // Diagnostic message
    // cout << "New X/Y/T: {" << particles[i].x << ", " << particles[i].y << ", "
    // 		 << particles[i].theta << "}" << endl;

  }

}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {

	// Empty weights vector from last update
	weights.clear();

	for (unsigned int i = 0; i < particles.size(); i++) {

		// cout << "working on Particle #" << i << endl;

		// For debug
	  vector<int> associations;
	  vector<double> sense_x;
	  vector<double> sense_y;

		// Reset and retrieve particle values
		double weight = 1;
		double x_part = particles[i].x;
		double y_part = particles[i].y;
		double theta = particles[i].theta;

		// Transform all observed landmark locations (reported in vehicle space) to
		// a set of predicted landmark locations (reported in map space)
		vector<LandmarkObs> predicted;
		for (unsigned int j = 0; j < observations.size(); j++) {
			// cout << "working on Particle #" << i << ", Obs #" << j+1 << endl;

			// Get x/y from each observation
			double x_obs = observations[j].x;
			double y_obs = observations[j].y;

			// cout << "x_obs, y_obs = " << x_obs << "," << y_obs << endl;

			// Transform to map x/y coordinates
			double x_map, y_map;
			x_map = x_part + (cos(theta) * x_obs) - (sin(theta) * y_obs);
			y_map = y_part + (sin(theta) * x_obs) + (cos(theta) * y_obs);

			// cout << "x_map, y_map = " << x_map << "," << y_map << endl;

			// Nearest Neighbor search to assign observation to a known landmark
			double min_dist_sq = sensor_range*sensor_range;
			double ux, uy;
			int assoc;

			// cout << "Start nearest neighbor search" << endl;

			// Iterate through landmark locations to find smallest distance
			// Using (distance^2) for calcs avoids call to sqrt()
			for (unsigned int k = 0; k < map_landmarks.landmark_list.size(); k++) {

				// cout << "Checking neighbor #" << k << endl;

				double landmark_x = map_landmarks.landmark_list[k].x_f;
				double landmark_y = map_landmarks.landmark_list[k].y_f;

				double x_delta = x_map - landmark_x;
				double y_delta = y_map - landmark_y;

				// cout << "x_delta, y_delta = " << x_delta << "," << y_delta << endl;

				double dist_sq = x_delta*x_delta + y_delta*y_delta;

				// If a new "smallest distance so far" is found, update the value and
				// update the ID that the observation points to
				if (dist_sq < min_dist_sq) {
					// cout << "updating min_dist_sq to " << dist_sq << endl;
					min_dist_sq = dist_sq;
					ux = landmark_x;
					uy = landmark_y;
					assoc = map_landmarks.landmark_list[k].id_i;
				}

			}

			// Implement probability density equation
			double std_x = std_landmark[0];
			double std_y = std_landmark[1];
			double obs_w = exp(-(x_map-ux)*(x_map-ux)/(2*std_x*std_x)
											-(y_map-uy)*(y_map-uy)/(2*std_y*std_y))
											/(2*M_PI*std_x*std_y);

			// cout << "obs_weight = " << obs_w << endl;

			// Multiply all observation weights together to get particle weight
			weight *= obs_w;

			// For debug
			associations.push_back(assoc);
			sense_x.push_back(ux);
			sense_y.push_back(uy);

		}

		// cout << "updated particle weight = " << weight << endl << endl;

		// Update particle weight and entry in the weights vector
		particles[i].weight = weight;
		weights.push_back(weight);

		// For debug
		SetAssociations(particles[i], associations, sense_x, sense_y);

	}

}

void ParticleFilter::resample() {

	vector<Particle> new_particles;

	// Set up resampling wheel
	default_random_engine gen;
	uniform_real_distribution<double> dist(0.0, num_particles);
	int index = dist(gen);
	double beta = 0.0;
	double max_w = *max_element(weights.begin(), weights.end());

	for (int i = 0; i < num_particles; i++) {
		beta += dist(gen) * 2 * max_w;
		while (weights[index] < beta) {
			beta -= weights[index];
			index = (index + 1) % num_particles;		// iterate index, but loop if > N
		}

		new_particles.push_back(particles[index]);

    // Diagnostic message
    // cout << "New X/Y/T: {" << particles[index].x << ", "
    // 		 << particles[index].y << ", " << particles[index].theta << "}" << endl;
	}

	particles = new_particles;

}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}
