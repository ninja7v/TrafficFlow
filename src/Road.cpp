// Libraries
#include <GLFW/glfw3.h> // To display
#include <memory>       // To use smart pointers
#include <cmath>        // To use sqrt
#include <list>         // To use lists
// Headers
#include "../headers/Road.h"
#include "../headers/Map.h"
#include "../headers/Constants.h"

Road::Road(const int id,
           Intersection* begin,
           Intersection* end)
   : idRoad(id),
     i1(begin),
     i2(end),
     length(sqrt(pow(begin->getPosition()[0] - end->getPosition()[0], 2) +
                 pow(begin->getPosition()[1] - end->getPosition()[1], 2))),
     direction{ (end->getPosition()[0] - begin->getPosition()[0]) / length,
                (end->getPosition()[1] - begin->getPosition()[1]) / length },
     roadCoordinates{ direction[1] * constants::halfWidthRoad + i1->getPosition()[0] * constants::ratioX + constants::margin,
                     -direction[0] * constants::halfWidthRoad + i1->getPosition()[1] * constants::ratioY + constants::margin, 0.0,
                      direction[1] * constants::halfWidthRoad + i2->getPosition()[0] * constants::ratioX + constants::margin,
                     -direction[0] * constants::halfWidthRoad + i2->getPosition()[1] * constants::ratioY + constants::margin, 0.0 },
     sideLeft{ i1->getPosition()[0] * constants::ratioX + constants::margin,
               i1->getPosition()[1] * constants::ratioY + constants::margin, 0.0,
               i2->getPosition()[0] * constants::ratioX + constants::margin,
               i2->getPosition()[1] * constants::ratioY + constants::margin, 0.0 },
     sideRight{ direction[1] * constants::widthRoad + i1->getPosition()[0] * constants::ratioX + constants::margin,
               -direction[0] * constants::widthRoad + i1->getPosition()[1] * constants::ratioY + constants::margin, 0.0,
                direction[1] * constants::widthRoad + i2->getPosition()[0] * constants::ratioX + constants::margin,
               -direction[0] * constants::widthRoad + i2->getPosition()[1] * constants::ratioY + constants::margin, 0.0 },
     lightCoordinates{ direction[1] * constants::halfWidthRoad + (i2->getPosition()[0] - direction[0] * 1.0) * constants::ratioX + constants::margin,
                      -direction[0] * constants::halfWidthRoad + (i2->getPosition()[1] - direction[1] * 1.0) * constants::ratioY + constants::margin } {
}

Road::~Road(){}

bool Road::containVehicle() const {
   return !Vehicles.empty();
}

void Road::addVehicle(std::shared_ptr<Vehicle> v) {
   if (v)
   {
      Vehicles.push_back(v);
      v->setPosition(i1->getPosition());
      v->setDirection(i2);
      v->resetRoadWaitingTime();
   }
}

double Road::getFreeLengthAtStart() const {
   if (Vehicles.empty())
      return length;
   const auto& last = Vehicles.back();
   const std::vector<double>& p = last->getPosition();
   const std::vector<double>& s = i1->getPosition();
   // Vehicle sizes are in pixels; the car-following model converts them with a factor 1/7.
   return std::hypot(p[0] - s[0], p[1] - s[1]) - last->getHeight() / 7.0;
}

bool Road::canEnter(const Vehicle& v) const {
   return getFreeLengthAtStart() - v.getHeight() / 7.0 > constants::distanceSecurity;
}

bool Road::isFull() const {
   return getFreeLengthAtStart() - constants::heightCar / 7.0 <= constants::distanceSecurity;
}

int Road::getNumberOfDepartedVehicles() const {
   return departedVehicles;
}

int Road::getQueueLength() const {
   int queue = 0;
   for (const auto& v : Vehicles)
      queue += v->isWaiting();
   return queue;
}

double Road::getHeadWaitingTime() const {
   return Vehicles.empty() ? 0.0 : Vehicles.front()->getRoadWaitingTime();
}

void Road::removeVehicle() {
   Vehicles.pop_front();
   departedVehicles++;
}

void Road::moveVehicles() {
   if (containVehicle()) {
      // For the first car
      std::shared_ptr<Vehicle> v = Vehicles.front();
      if (v)
      {
         const bool atIntersection = v->distance(i2) < constants::distanceSecurity / 5.0;
         if (atIntersection) {
            if (!v->nextRoad() && v->getDestination() != i2) {
               // No itinerary but not at destination (e.g. no path found at spawn): try again from here.
               v->setNewItinerary(Map::track(i2, v->getDestination(), this));
            }
            if (!v->nextRoad()) {
               this->removeVehicle();
               v->setStatus(true);
            }
            else {
               if (v->nextRoad()->canEnter(*v)) {
                  v->nextRoad()->addVehicle(v);
                  v->updateItinerary();
                  this->removeVehicle();
               }
               else {
                  // Next road is full: wait, and look for another route from the intersection we are at.
                  v->stop();
                  v->setNewItinerary(Map::track(i2, v->getDestination(), this));
               }
            }
         }
         else
            v->moveToIntersection(i2, idRoad);
      }
      // For the folowing cars
      // We ignore the case atIntersection = true, were the first car is gone
      if (Vehicles.size() > 1) {
         v = *Vehicles.begin();
         for (auto it = std::next(Vehicles.begin()); it != Vehicles.end(); ++it){
            if (*it)
            {
               (*it)->moveToVehicle(v);
               v = *it;
            }
         }
      }
      for (const auto& vehicle : Vehicles)
         vehicle->updateWaitingTime();
   }
}

void Road::displayRoad() const {
   glEnable(GL_LINE_SMOOTH);
   glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
   glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
   glEnableClientState(GL_VERTEX_ARRAY);
   glScaled(1.0, 1.0, 0.0);
   // Road
   glColor3d(0.3, 0.3, 0.3); // Grey
   glLineWidth(static_cast<float>(constants::widthRoad));
   glVertexPointer(3, GL_DOUBLE, 0, roadCoordinates);
   glDrawArrays(GL_LINES, 0, 2);
   // Sides
   glColor3d(1.0, 1.0, 1.0); // White
   glLineWidth(static_cast<float>(constants::widthRoad) / 10.0f);
   glVertexPointer(3, GL_DOUBLE, 0, sideLeft);
   glDrawArrays(GL_LINES, 0, 2);
   glVertexPointer(3, GL_DOUBLE, 0, sideRight);
   glDrawArrays(GL_LINES, 0, 2);
   glDisableClientState(GL_VERTEX_ARRAY);
   glDisable(GL_LINE_SMOOTH);
}

void Road::displayLight() const {
   // Outline
   glColor3d(0.0, 0.0, 0.0); // Black
   glPointSize(static_cast<float>(constants::widthRoad) * 0.7f);
   glEnable(GL_POINT_SMOOTH);
   glBegin(GL_POINTS);
   glVertex2d(lightCoordinates[0], lightCoordinates[1]);
   glEnd();
   // Outline without overlay
   //const int nStep = 10; // # of triangles used to draw the circle
   //const GLfloat step = 6.283f / nStep;
   //glLineWidth(constants::widthRoad / 5);
   //glColor3d(0.0, 0.0, 0.0); // Black
   //glBegin(GL_LINE_LOOP);
   //for (int i = 0; i <= nStep; i++)
   //   glVertex2f(lightCoordinates[0] + (diameter * cos(i * step) / 2),
   //              lightCoordinates[1] + (diameter * sin(i * step) / 2)
   //   );
   //glEnd();
   // Inside
   if (i2->isRed(idRoad))
      glColor3d(1.0, 0.0, 0.0); // Red
   else
      glColor3d(0.0, 1.0, 0.0); // Green
   glPointSize(static_cast<float>(constants::halfWidthRoad));
   glBegin(GL_POINTS);
   glVertex2f(static_cast<float>(lightCoordinates[0]), static_cast<float>(lightCoordinates[1]));
   glEnd();
   glDisable(GL_POINT_SMOOTH);
}

int Road::getID() const {
   return idRoad;
}

double Road::getLength() const {
   return length;
}

Intersection* Road::getStart() const {
   return i1;
}

Intersection* Road::getEnd() const {
   return i2;
}

const std::list<std::shared_ptr<Vehicle>>& Road::getVehicles() const {
   return Vehicles;
}

std::array<double, 2> Road::getDirection() const {
   return direction;
}
