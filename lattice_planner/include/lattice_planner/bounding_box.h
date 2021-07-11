class Box2d
{
public:
	// Box2d() = default;

  	Box2d(const Vec2d &center, const double heading, const double length,
        const double width):
  	  center_(center),
      length_(length),
      width_(width),
      half_length_(length / 2.0),
      half_width_(width / 2.0),
      heading_(heading),
      cos_heading_(cos(heading)),
      sin_heading_(sin(heading)){
        InitCorners();
      };

	// ~Box2d();

    void InitCorners() {
      const double dx1 = cos_heading_ * half_length_;
      const double dy1 = sin_heading_ * half_length_;
      const double dx2 = sin_heading_ * half_width_;
      const double dy2 = -cos_heading_ * half_width_;
      corners_.clear();
      ///////////////////   why???    ///////////////////////////////////////
      
      Vec2d corner1 = {center_.x + dx1 + dx2, center_.y + dy1 + dy2};
      Vec2d corner2 = {center_.x + dx1 - dx2, center_.y + dy1 - dy2};
      Vec2d corner3 = {center_.x - dx1 - dx2, center_.y - dy1 - dy2};
      Vec2d corner4 = {center_.x - dx1 + dx2, center_.y - dy1 + dy2};

      corners_.emplace_back( corner1 );
      corners_.emplace_back( corner2 );
      corners_.emplace_back( corner3 );
      corners_.emplace_back( corner4 );

      for (auto &corner : corners_) {
        max_x_ = std::fmax(corner.x, max_x_);
        min_x_ = std::fmin(corner.x, min_x_);
        max_y_ = std::fmax(corner.y, max_y_);
        min_y_ = std::fmin(corner.y, min_y_);
      }
    }

	void Shift(const Vec2d &shift_vec) {
	  // center_ += shift_vec;
		center_.x += shift_vec.x;
		center_.y += shift_vec.y;
	}

	bool HasOverlap(const Box2d &box) const {
	  if (box.max_x() < min_x() || box.min_x() > max_x() || box.max_y() < min_y() ||
	      box.min_y() > max_y()) {
	    return false;
	  }

	  const double shift_x = box.center_x() - center_.x;
	  const double shift_y = box.center_y() - center_.y;

	  const double dx1 = cos_heading_ * half_length_;
	  const double dy1 = sin_heading_ * half_length_;
	  const double dx2 = sin_heading_ * half_width_;
	  const double dy2 = -cos_heading_ * half_width_;
	  const double dx3 = box.cos_heading() * box.half_length();
	  const double dy3 = box.sin_heading() * box.half_length();
	  const double dx4 = box.sin_heading() * box.half_width();
	  const double dy4 = -box.cos_heading() * box.half_width();

	  return std::abs(shift_x * cos_heading_ + shift_y * sin_heading_) <=
	             std::abs(dx3 * cos_heading_ + dy3 * sin_heading_) +
	                 std::abs(dx4 * cos_heading_ + dy4 * sin_heading_) +
	                 half_length_ &&
	         std::abs(shift_x * sin_heading_ - shift_y * cos_heading_) <=
	             std::abs(dx3 * sin_heading_ - dy3 * cos_heading_) +
	                 std::abs(dx4 * sin_heading_ - dy4 * cos_heading_) +
	                 half_width_ &&
	         std::abs(shift_x * box.cos_heading() + shift_y * box.sin_heading()) <=
	             std::abs(dx1 * box.cos_heading() + dy1 * box.sin_heading()) +
	                 std::abs(dx2 * box.cos_heading() + dy2 * box.sin_heading()) +
	                 box.half_length() &&
	         std::abs(shift_x * box.sin_heading() - shift_y * box.cos_heading()) <=
	             std::abs(dx1 * box.sin_heading() - dy1 * box.cos_heading()) +
	                 std::abs(dx2 * box.sin_heading() - dy2 * box.cos_heading()) +
	                 box.half_width();
	}

	const Vec2d &center() const { return center_; }
	  double center_x() const { return center_.x; }

	  /**
	   * @brief Getter of the y-coordinate of the center of the box
	   * @return The y-coordinate of the center of the box
	   */
	  double center_y() const { return center_.y; }

	  /**
	   * @brief Getter of the length
	   * @return The length of the heading-axis
	   */
	  double length() const { return length_; }

	  /**
	   * @brief Getter of the width
	   * @return The width of the box taken perpendicularly to the heading
	   */
	  double width() const { return width_; }

	  /**
	   * @brief Getter of half the length
	   * @return Half the length of the heading-axis
	   */
	  double half_length() const { return half_length_; }

	  /**
	   * @brief Getter of half the width
	   * @return Half the width of the box taken perpendicularly to the heading
	   */
	  double half_width() const { return half_width_; }

	  /**
	   * @brief Getter of the heading
	   * @return The counter-clockwise angle between the x-axis and the heading-axis
	   */
	  double heading() const { return heading_; }

	  /**
	   * @brief Getter of the cosine of the heading
	   * @return The cosine of the heading
	   */
	  double cos_heading() const { return cos_heading_; }

	  /**
	   * @brief Getter of the sine of the heading
	   * @return The sine of the heading
	   */
	  double sin_heading() const { return sin_heading_; }


  	double max_x() const { return max_x_; }
  	double min_x() const { return min_x_; }
  	double max_y() const { return max_y_; }
  	double min_y() const { return min_y_; }
private:
  	Vec2d center_;
  	double length_ = 0.0;
  	double width_ = 0.0;
  	double half_length_ = 0.0;
  	double half_width_ = 0.0;
  	double heading_ = 0.0;
  	double cos_heading_ = 1.0;
  	double sin_heading_ = 0.0;

  	std::vector<Vec2d> corners_;

  	// double max_x_ = std::numeric_limits<double>::min();
		double max_x_ = std::numeric_limits<double>::lowest();
  	double min_x_ = std::numeric_limits<double>::max();
  	// double max_y_ = std::numeric_limits<double>::min();
		double max_y_ = std::numeric_limits<double>::lowest();
  	double min_y_ = std::numeric_limits<double>::max();
	
};