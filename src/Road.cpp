// Libraries
#include <GLFW/glfw3.h> // To display
#include <ctime>        // To use clock() and clock_t
#include <memory>       // To use smart pointers
#include <cmath>        // To use sqrt
#include <list>         // To use lists
// Headers
#include "../headers/Global.h"
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
     totalNumberOfArringVehicles(0),
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
      v->setEnterRoadTime(clock());
      totalNumberOfArringVehicles += 1;
   }
}

void Road::removeVehicle() {
   Vehicles.pop_front();
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
               v->setNewItinerary(Map::track(i2, v->getDestination()));
            }
            if (!v->nextRoad()) {
               this->removeVehicle();
               v->setStatus(true);
               global::numberOfVehicles -= 1;
            }
            else {
               const bool isEnoughSpace = !v->nextRoad()->containVehicle() ||
                  v->nextRoad()->getVehicles().back()->distance(i2) > constants::distanceSecurity + v->getHeight();
               if (isEnoughSpace) {
                  v->nextRoad()->addVehicle(v);
                  v->updateItinerary();
                  this->removeVehicle();
               }
               else {
                  // Next road is full: look for another route from the intersection we are at.
                  v->setNewItinerary(Map::track(i2, v->getDestination()));
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

int Road::getTotalNumberOfArringVehicles() const {
   return totalNumberOfArringVehicles;
}

std::tuple<int, int, int> Road::getVehicleStats(double averageNewVehicles) const {
   // Occupancy: 0=Empty, 1=Low, 2=High
   const size_t nbVehicles = Vehicles.size();
   const int occupancy = (nbVehicles < 1) ? 0 :
                         (nbVehicles < 7) ? 1 : 2;

   // First Car Speed: 0=Stopped, 1=Medium, 2=Fast
   int speedState = 0; // Default to 1 if there are cars
   if (!Vehicles.empty()) {
       const double s = Vehicles.front()->getSpeed();
       speedState = (s < constants::speedMaxCar * 0.1) ? 0 :
                    (s < constants::speedMaxCar * 0.8) ? 1 : 2;
   }

   // Usage: 0=Few, 1=Medium, 2=Crowded
   const double ratio = (averageNewVehicles > 0) ? totalNumberOfArringVehicles / averageNewVehicles : 1.0;
   const int usage = ratio < 0.5 ? 0 :
                     ratio < 1.5 ? 1 : 2;

   return std::make_tuple(occupancy, speedState, usage);
}
