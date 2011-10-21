

#ifndef IIDGENERATOR_H
#define IIDGENERATOR_H

namespace BRICS_3D {

namespace RSG {

class IIdGenerator {
  public:
    virtual unsigned int getNextValidId() = 0;

};

} // namespace BRICS_3D::RSG

} // namespace BRICS_3D
#endif

/* EOF */

