double NormalizeAngle(double x){
		// TODO: to makesure the function means;
		return x;
}

double slerp(const double a0, const double t0, const double a1, const double t1,
	             const double t) {
	  if (std::abs(t1 - t0) <= kMathEpsilon) {
	    // AERROR << "input time difference is too small";
	    return NormalizeAngle(a0);
	  }
	  const double a0_n = NormalizeAngle(a0);
	  const double a1_n = NormalizeAngle(a1);
	  double d = a1_n - a0_n;
	  if (d > M_PI) {
	    d = d - 2 * M_PI;
	  } else if (d < -M_PI) {
	    d = d + 2 * M_PI;
	  }

	  const double r = (t - t0) / (t1 - t0);
	  const double a = a0_n + d * r;
	  return NormalizeAngle(a);
}
//返回值类型为PathPoint(结构体)类型的函数
PathPoint InterpolateUsingLinearApproximation(const PathPoint &p0,const PathPoint &p1,const double s)
{
	  double s0 = p0.s;
	  double s1 = p1.s;
	  // CHECK_LE(s0, s1);

	  PathPoint path_point;
	  double weight = (s - s0) / (s1 - s0);
	  double x = (1 - weight) * p0.x + weight * p1.x;
	  double y = (1 - weight) * p0.y + weight * p1.y;
	  double theta = slerp(p0.theta, p0.s, p1.theta, p1.s, s);
	  double kappa = (1 - weight) * p0.kappa + weight * p1.kappa;
	  double dkappa = (1 - weight) * p0.dkappa + weight * p1.dkappa;
	  // double ddkappa = (1 - weight) * p0.ddkappa + weight * p1.ddkappa;

	  path_point.x = x;
	  path_point.y = y;
	  path_point.theta = theta;
	  path_point.kappa = kappa;
	  path_point.dkappa = dkappa;
	  // path_point.ddkappa = ddkappa;
	  path_point.s = s;
	  return path_point;
}

PathPoint MatchToPath(const std::vector<PathPoint>& reference_line,const double s) 
{
	  auto comp = [](const PathPoint& point, const double s) 
	  {
	    return point.s < s;
	  };

	  auto it_lower =
	      std::lower_bound(reference_line.begin(), reference_line.end(), s, comp);
	  if (it_lower == reference_line.begin()) {
	    return reference_line.front();
	  } else if (it_lower == reference_line.end()) {
	    return reference_line.back();
	  }

	  // interpolate between it_lower - 1 and it_lower
	  // return interpolate(*(it_lower - 1), *it_lower, s);
	  return InterpolateUsingLinearApproximation(*(it_lower - 1), *it_lower, s);
}

void frenet_to_cartesian(
	const double rs, const double rx, const double ry, const double rtheta,
	const double rkappa, const double rdkappa,
	const std::array<double, 3>& s_condition,
	const std::array<double, 3>& d_condition, double* const ptr_x,
	double* const ptr_y, double* const ptr_theta, double* const ptr_kappa,
	double* const ptr_v, double* const ptr_a) {

		const double cos_theta_r = std::cos(rtheta);
		const double sin_theta_r = std::sin(rtheta);

		/////////////  d is positive on the left
		*ptr_x = rx - sin_theta_r * d_condition[0];
		*ptr_y = ry + cos_theta_r * d_condition[0];

		const double one_minus_kappa_r_d = 1 - rkappa * d_condition[0];

		const double tan_delta_theta = d_condition[1] / one_minus_kappa_r_d;
		const double delta_theta = std::atan2(d_condition[1], one_minus_kappa_r_d);
		const double cos_delta_theta = std::cos(delta_theta);

		*ptr_theta = NormalizeAngle(delta_theta + rtheta);

		const double kappa_r_d_prime =
		      rdkappa * d_condition[0] + rkappa * d_condition[1];
		*ptr_kappa = (((d_condition[2] + kappa_r_d_prime * tan_delta_theta) *
		                 cos_delta_theta * cos_delta_theta) /
		                    (one_minus_kappa_r_d) +
		                rkappa) *
		               cos_delta_theta / (one_minus_kappa_r_d);

		const double d_dot = d_condition[1] * s_condition[1];
		*ptr_v = std::sqrt(one_minus_kappa_r_d * one_minus_kappa_r_d *
		                         s_condition[1] * s_condition[1] +
		                     d_dot * d_dot);

		const double delta_theta_prime =
		      one_minus_kappa_r_d / cos_delta_theta * (*ptr_kappa) - rkappa;

		*ptr_a = s_condition[2] * one_minus_kappa_r_d / cos_delta_theta +
		           s_condition[1] * s_condition[1] / cos_delta_theta *
	               (d_condition[1] * delta_theta_prime - kappa_r_d_prime);
}

void cartesian_to_frenet(
    const double rs, const double rx, const double ry, const double rtheta,
    const double rkappa, const double rdkappa, const double x, const double y,
    const double v, const double a, const double theta, const double kappa,
    std::array<double, 3>* const ptr_s_condition,
    std::array<double, 3>* const ptr_d_condition) {
  const double dx = x - rx;
  const double dy = y - ry;

  const double cos_theta_r = std::cos(rtheta);
  const double sin_theta_r = std::sin(rtheta);

  const double cross_rd_nd = cos_theta_r * dy - sin_theta_r * dx;
  ptr_d_condition->at(0) =
      std::copysign(std::sqrt(dx * dx + dy * dy), cross_rd_nd);
  // printf("dx, dy, rtheta, cross_rd_nd, ptr_d_condition->at(0) %f %f %f %f %f \n", dx, dy, rtheta, cross_rd_nd, ptr_d_condition->at(0));
  const double delta_theta = theta - rtheta;
  const double tan_delta_theta = std::tan(delta_theta);
  const double cos_delta_theta = std::cos(delta_theta);

  const double one_minus_kappa_r_d = 1 - rkappa * ptr_d_condition->at(0);
  ptr_d_condition->at(1) = one_minus_kappa_r_d * tan_delta_theta;

  const double kappa_r_d_prime =
      rdkappa * ptr_d_condition->at(0) + rkappa * ptr_d_condition->at(1);

  ptr_d_condition->at(2) =
      -kappa_r_d_prime * tan_delta_theta +
      one_minus_kappa_r_d / cos_delta_theta / cos_delta_theta *
          (kappa * one_minus_kappa_r_d / cos_delta_theta - rkappa);

  ptr_s_condition->at(0) = rs;

  ptr_s_condition->at(1) = v * cos_delta_theta / one_minus_kappa_r_d;

  const double delta_theta_prime =
      one_minus_kappa_r_d / cos_delta_theta * kappa - rkappa;
  ptr_s_condition->at(2) =
      (a * cos_delta_theta -
       ptr_s_condition->at(1) * ptr_s_condition->at(1) *
           (ptr_d_condition->at(1) * delta_theta_prime - kappa_r_d_prime)) /
      one_minus_kappa_r_d;
  return;
}

void cartesian_to_frenet(
    const double rs, const double rx, const double ry, const double rtheta,
    const double x, const double y, double* ptr_s, double* ptr_d) {
  const double dx = x - rx;
  const double dy = y - ry;

  const double cos_theta_r = std::cos(rtheta);
  const double sin_theta_r = std::sin(rtheta);

  const double cross_rd_nd = cos_theta_r * dy - sin_theta_r * dx;
  *ptr_d = std::copysign(std::sqrt(dx * dx + dy * dy), cross_rd_nd);
  *ptr_s = rs;
  return;
}