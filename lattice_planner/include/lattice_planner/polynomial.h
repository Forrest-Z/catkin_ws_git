class Quartic_Polynomial
{
public:
	Quartic_Polynomial(double x0, double dx0, double ddx0,
						double dx1, double ddx1, double param){
		ComputeCoefficients(x0, dx0, ddx0, dx1, ddx1, param);
	}
	// ~Quartic_Polynomial();
	
	

	double Evaluate(const std::uint32_t order, const double p) const {
	  switch (order) { //order:几阶导数
	    case 0: {
	      return (((coef_[4] * p + coef_[3]) * p + coef_[2]) * p + coef_[1]) * p +
	             coef_[0];
	    }
	    case 1: {
	      return ((4.0 * coef_[4] * p + 3.0 * coef_[3]) * p + 2.0 * coef_[2]) * p +
	             coef_[1];
	    }
	    case 2: {
	      return (12.0 * coef_[4] * p + 6.0 * coef_[3]) * p + 2.0 * coef_[2];
	    }
	    case 3: {
	      return 24.0 * coef_[4] * p + 6.0 * coef_[3];
	    }
	    case 4: {
	      return 24.0 * coef_[4];
	    }
	    default:
	      return 0.0;
	  }
	}


private:
	void ComputeCoefficients(
	    const double x0, const double dx0, const double ddx0, const double dx1,
	    const double ddx1, const double p) {

	  coef_[0] = x0;
	  coef_[1] = dx0;
	  coef_[2] = 0.5 * ddx0;

	  double b0 = dx1 - ddx0 * p - dx0;
	  double b1 = ddx1 - ddx0;

	  double p2 = p * p;
	  double p3 = p2 * p;

	  coef_[3] = (3 * b0 - b1 * p) / (3 * p2);
	  coef_[4] = (-2 * b0 + b1 * p) / (4 * p3);

	  return;
	}

	std::array<double, 5> coef_ = {{0.0, 0.0, 0.0, 0.0, 0.0}};
	// double param;
	// double x0, dx0, ddx0, dx1, ddx1;
};


class Quintic_Polynomial
{
public:
	Quintic_Polynomial(double x0, double dx0, double ddx0,
					double x1, double dx1, double ddx1, double param)
	{
		ComputeCoefficients(x0, dx0, ddx0, x1, dx1, ddx1, param);
	}
	// ~Quintic_Polynomial();
	
	

	double Evaluate(const uint32_t order, const double p) const
	 {
	  switch (order) {
	    case 0: {
	      return ((((coef_[5] * p + coef_[4]) * p + coef_[3]) * p + coef_[2]) * p +
	              coef_[1]) *p +coef_[0];
	    }
	    case 1: {
	      return (((5.0 * coef_[5] * p + 4.0 * coef_[4]) * p + 3.0 * coef_[3]) * p +
	              2.0 * coef_[2]) *p +coef_[1];
	    }
	    case 2: {
	      return (((20.0 * coef_[5] * p + 12.0 * coef_[4]) * p) + 6.0 * coef_[3]) *p +
	             2.0 * coef_[2];
	    }
	    case 3: {
	      return (60.0 * coef_[5] * p + 24.0 * coef_[4]) * p + 6.0 * coef_[3];
	    }
	    case 4: {
	      return 120.0 * coef_[5] * p + 24.0 * coef_[4];
	    }
	    case 5: {
	      return 120.0 * coef_[5];
	    }
	    default:
	      return 0.0;
		}
	}


private:
	void ComputeCoefficients(
	    const double x0, const double dx0, const double ddx0, const double x1,
	    const double dx1, const double ddx1, const double p) 
	{
		coef_[0] = x0;
		coef_[1] = dx0;
		coef_[2] = ddx0 / 2.0;

		const double p2 = p * p;
		const double p3 = p * p2;

		// the direct analytical method is at least 6 times faster than using matrix
		// inversion.
		const double c0 = (x1 - 0.5 * p2 * ddx0 - dx0 * p - x0) / p3;
		const double c1 = (dx1 - ddx0 * p - dx0) / p2;
		const double c2 = (ddx1 - ddx0) / p;

		coef_[3] = 0.5 * (20.0 * c0 - 8.0 * c1 + c2);
		coef_[4] = (-15.0 * c0 + 7.0 * c1 - c2) / p;
		coef_[5] = (6.0 * c0 - 3.0 * c1 + 0.5 * c2) / p2;
	}

	std::array<double, 6> coef_ = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
	// double param;
	// double x0, dx0, ddx0, x1, dx1, ddx1;
};