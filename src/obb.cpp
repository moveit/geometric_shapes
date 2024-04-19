#include <geometric_shapes/obb.h>

#include <fcl/config.h>

#if FCL_MAJOR_VERSION == 0 && FCL_MINOR_VERSION == 5
#include <fcl/BV/OBB.h>
typedef fcl::Vec3f FCL_Vec3;
typedef fcl::OBB FCL_OBB;
#else
#include <fcl/math/bv/OBB.h>
typedef fcl::Vector3d FCL_Vec3;
typedef fcl::OBB<double> FCL_OBB;
#endif

namespace bodies
{
#if FCL_MAJOR_VERSION == 0 && FCL_MINOR_VERSION == 5
inline FCL_Vec3 toFcl(const Eigen::Vector3d& eigenVec)
{
  FCL_Vec3 result;
  Eigen::Map<Eigen::MatrixXd>(result.data.vs, 3, 1) = eigenVec;
  return result;
}
#else
#define toFcl
#endif

#if FCL_MAJOR_VERSION == 0 && FCL_MINOR_VERSION == 5
Eigen::Vector3d fromFcl(const FCL_Vec3& fclVec)
{
  return Eigen::Map<const Eigen::MatrixXd>(fclVec.data.vs, 3, 1);
}
#else
#define fromFcl
#endif

class OBBPrivate : public FCL_OBB
{
public:
  using FCL_OBB::OBB;
};

OBB::OBB()
{
  obb_.reset(new OBBPrivate);
  // Initialize the OBB to position 0, with 0 extents and identity rotation
#if FCL_MAJOR_VERSION > 0 || FCL_MINOR_VERSION > 5
  // FCL 0.6+ does not zero-initialize the OBB.
  obb_->extent.setZero();
  obb_->To.setZero();
  obb_->axis.setIdentity();
#else
  // FCL 0.5 zero-initializes the OBB, so we just put the identity into orientation.
  obb_->axis[0][0] = 1.0;
  obb_->axis[1][1] = 1.0;
  obb_->axis[2][2] = 1.0;
#endif
}

OBB::OBB(const OBB& other)
{
  obb_.reset(new OBBPrivate(*other.obb_));
}

OBB::OBB(const Eigen::Isometry3d& pose, const Eigen::Vector3d& extents)
{
  obb_.reset(new OBBPrivate);
  setPoseAndExtents(pose, extents);
}

OBB& OBB::operator=(const OBB& other)
{
  *obb_ = *other.obb_;
  return *this;
}

void OBB::setPoseAndExtents(const Eigen::Isometry3d& pose, const Eigen::Vector3d& extents)
{
  const auto rotation = pose.linear();

#if FCL_MAJOR_VERSION == 0 && FCL_MINOR_VERSION == 5
  obb_->axis[0] = toFcl(rotation.col(0));
  obb_->axis[1] = toFcl(rotation.col(1));
  obb_->axis[2] = toFcl(rotation.col(2));
#else
  obb_->axis = rotation;
#endif

  obb_->To = toFcl(pose.translation());

  obb_->extent = { extents[0] / 2.0, extents[1] / 2.0, extents[2] / 2.0 };
}

void OBB::getExtents(Eigen::Vector3d& extents) const
{
  extents = 2 * fromFcl(obb_->extent);
}

Eigen::Vector3d OBB::getExtents() const
{
  Eigen::Vector3d extents;
  getExtents(extents);
  return extents;
}

void OBB::getPose(Eigen::Isometry3d& pose) const
{
  pose = Eigen::Isometry3d::Identity();
  pose.translation() = fromFcl(obb_->To);
#if FCL_MAJOR_VERSION == 0 && FCL_MINOR_VERSION == 5
  pose.linear().col(0) = fromFcl(obb_->axis[0]);
  pose.linear().col(1) = fromFcl(obb_->axis[1]);
  pose.linear().col(2) = fromFcl(obb_->axis[2]);
#else
  pose.linear() = obb_->axis;
#endif
}

Eigen::Isometry3d OBB::getPose() const
{
  Eigen::Isometry3d pose;
  getPose(pose);
  return pose;
}

AABB OBB::toAABB() const
{
  AABB result;
  toAABB(result);
  return result;
}

void OBB::toAABB(AABB& aabb) const
{
  aabb.extendWithTransformedBox(getPose(), getExtents());
}

OBB* OBB::extendApprox(const OBB& box)
{
  if (this->getExtents() == Eigen::Vector3d::Zero())
  {
    *obb_ = *box.obb_;
    return this;
  }

  if (this->contains(box))
    return this;

  if (box.contains(*this))
  {
    *obb_ = *box.obb_;
    return this;
  }

  *this->obb_ += *box.obb_;
  return this;
}

bool OBB::contains(const Eigen::Vector3d& point) const
{
  return obb_->contain(toFcl(point));
}

bool OBB::overlaps(const bodies::OBB& other) const
{
  return obb_->overlap(*other.obb_);
}

EigenSTL::vector_Vector3d OBB::computeVertices() const
{
  const Eigen::Vector3d e = getExtents() / 2;  // do not use auto type, Eigen would be inefficient
  // clang-format off
  EigenSTL::vector_Vector3d result = {
    { -e[0], -e[1], -e[2] },
    { -e[0], -e[1],  e[2] },
    { -e[0],  e[1], -e[2] },
    { -e[0],  e[1],  e[2] },
    {  e[0], -e[1], -e[2] },
    {  e[0], -e[1],  e[2] },
    {  e[0],  e[1], -e[2] },
    {  e[0],  e[1],  e[2] },
  };
  // clang-format on

  const auto pose = getPose();
  for (auto& v : result)
  {
    v = pose * v;
  }

  return result;
}

bool OBB::contains(const OBB& obb) const
{
  for (const auto& v : obb.computeVertices())
  {
    if (!contains(v))
      return false;
  }
  return true;
}

OBB::~OBB() = default;

}  // namespace bodies
