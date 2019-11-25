#include <cstdlib>
#include <iostream>
#include <math.h>
#include <vector>
#include <SFML/Graphics.hpp> // un seul include suffit pour avoir les trois parties essentielles
						     // de la SFML: graphics, window et system

using namespace std;
using namespace sf;

class space_object {

    private:

    	string name;
		double size;
        CircleShape shape;

        double x[3],y[3]; /*Canonical vectors, range 0 is the position of the object,
                    ranges above are the derivates in time
                    */
        
        double MaxSpeed;
        static int windowSizeX;
        static int windowSizeY;

        virtual void dynamics(int n, double t, double y[], double dy[]){

    		dy[0] = y[1];
    		dy[1] = y[2];

		}

    public:

        //Defined by partner for rotation mouvement rho and phy 
        double r,phy; //provisonal

        static int NumberOfSpaceObjects;

        string GetName(){ return name;}

        void SetName(string name){ this->name = name;}
        
        double GetSize(){ return size;}
        void SetSize(int size){ this->size = size;}

        CircleShape GetShape(){ return shape;}

        void draw (	RenderWindow &window){ window.draw(shape);}

        void SetLimits(int windowSizeX, int windowSizeY,double MaxSpeed){

            this->windowSizeX = windowSizeX;
            this->windowSizeY = windowSizeY;
            this->MaxSpeed = MaxSpeed;

        }
        int GetWindowSizeX(){ return windowSizeX;}
        int GetWindowSizeY(){ return windowSizeY;}

        //Clock to get the time since execution (it returns elapsed time in ms)
        double GetTickCount(){
                return clock_t()/double(CLOCKS_PER_SEC);
        }

        virtual void ApplyLimits(){
            /*
            This sets the limits of the space object
            */

            //Deffines Spacial limits
            if ( x[0] >= (windowSizeX-size) ){
                x[1]=-x[1];
                x[0]=windowSizeX-size;
            }

            if ( x[0] < size ) {
                x[1]=-x[1];
                x[0]=size;
            } 
        
            if ( y[0] >= (windowSizeY-size) ){
                y[1]=-y[1];
                y[0]=windowSizeY-size;
            }

            if ( y[0] < size ) {
                y[1]=-y[1];
                y[0]=size;
            }

            //Deffines Speed limits
            if ( x[1] > (MaxSpeed) ){
                x[1]=MaxSpeed;
            }

            if ( y[1] > (MaxSpeed) ){
                y[1]=MaxSpeed;
            }
            
            if ( x[1] < -(MaxSpeed) ){
                x[1]=-MaxSpeed;
            }

            if ( y[1] < -(MaxSpeed) ){
                y[1]=-MaxSpeed;
            }

        }

        double * GetVectorX(){ return x;}
        double * GetVectorY(){ return y;}
        void SetSpeed(double xSpeed,double ySpeed){
            this->x[1] = xSpeed;
            this->y[1] = ySpeed;
        }
        void SetForces(double xForce,double yForce){
            this->x[2] = xForce;
            this->y[2] = yForce;
        }

        double distance(space_object& A){

            double dx2 = pow((A.GetVectorX()[0]-GetVectorX()[0]), 2);
            double dy2 = pow((A.GetVectorY()[0]-GetVectorY()[0]), 2);

            return sqrt(dx2 + dy2);

        }

        
        void rk4(int n, double x, double y[], double dx){

            /*-----------------------------------------
            sous programme de resolution d'equations
            differentielles du premier ordre par
            la methode de Runge-Kutta d'ordre 4
            x = abscisse
            y = valeurs des fonctions
            dx = pas
            n = nombre d'equations differentielles
            dynamics = variable contenant le nom du
            sous-programme qui calcule les derivees
            ----------------------------------------*/

            int i ;
            double ddx ;
            /* d1, d2, d3, d4 = estimations des derivees
            yp = estimations intermediaires des fonctions */
            double d1[n], d2[n], d3[n], d4[n], yp[n];
            
            ddx = dx/2;                /* demi-pas */
            
            dynamics(n,x,y,d1) ;          /* 1ere estimation */          
            
            for( i = 0; i< n; i++){ yp[i] = y[i] + d1[i]*ddx ; }
            dynamics(n,x+ddx,yp,d2) ;     /* 2eme estimat. (1/2 pas) */
            
            for( i = 0; i < n; i++){ yp[i] = y[i] + d2[i]*ddx ; }
            dynamics(n,x+ddx,yp,d3) ; /* 3eme estimat. (1/2 pas) */
            
            for( i = 0; i< n; i++){ yp[i] = y[i] + d3[i]*dx ;}
            dynamics(n,x+dx,yp,d4) ;      /* 4eme estimat. (1 pas) */
            /* estimation de y pour le pas suivant en utilisant
            une moyenne pondérée des dérivées en remarquant
            que : 1/6 + 1/3 + 1/3 + 1/6 = 1 */
            
            for( i = 0; i < n ; i++)
                { y[i] = y[i] + dx*( d1[i] + 2*d2[i] + 2*d3[i] + d4[i] )/6 ; }
        }
        
        void UpdatePosition(){ 
            
            rk4(3, 1., x, 1e-1);
            rk4(3, 1., y, 1e-1);
            ApplyLimits();
            this->shape.setPosition(x[0],y[0]);

        }

        void SetNumberOfSpaceObjects (int input){ NumberOfSpaceObjects = NumberOfSpaceObjects; }
        static int GetNumberOfSpaceObjects (){ return NumberOfSpaceObjects; }

        void SetAll(string name, double size, int windowSizeX, int windowSizeY, double MaxSpeed , double x[], double y[]);
        void GetAll();

        //Constructors
        space_object(string name, int ShapePoints, Color color, double size, int windowSizeX, int windowSizeY, double MaxSpeed, double x[], double y[], double phy);
        space_object(int windowSizeX, int windowSizeY);

        //Destructor
        ~space_object(); // Frees up memory after objects are destroyed (To be defined - investigate)

};

//Instance of Static Variables

    int space_object::windowSizeX = 800;
    int space_object::windowSizeY = 600;
    int space_object::NumberOfSpaceObjects = 0;


//Methods related to Space Objects

    void space_object::SetAll(string name, double size, int windowSizeX, int windowSizeY, double MaxSpeed , double x[], double y[]){

        this->name = name;
        this->size = size;
        this->windowSizeX = windowSizeX;
        this->windowSizeY = windowSizeY;
        this->MaxSpeed = MaxSpeed;
        for(int i=0; i<3; i ++) this-> x[i] = x[i];
        for(int i=0; i<3; i ++) this-> y[i] = y[i];

    }

    void space_object::GetAll(){

        //Check control
        cout<< endl
            << "Name: " << name << endl
            <<  "WindowSizeX: " << windowSizeX << endl
            <<  "WindowSizeY: " << windowSizeY << endl
            <<  "MaxSpeed: "    << MaxSpeed << endl
            <<  "   Position: ("    << x[0]<< ","<<y[0]<<")"
            <<  "   Vitesse: ("     << x[1]<< ","<<y[1]<<")"
            <<  "   acceleration: ("<< x[2]<< ","<<y[2]<<")\n";
    }

    space_object::space_object(string name, int ShapePoints, Color color, double size, int windowSizeX, int windowSizeY, double MaxSpeed , double x[], double y[], double phy){

        this-> name = name;
        this-> size = size;
        this-> windowSizeX = windowSizeX;
        this-> windowSizeY = windowSizeY;
        this-> MaxSpeed = MaxSpeed;
        this-> phy = phy;
        for(int i=0; i<3; i ++) this-> x[i] = x[i];
        for(int i=0; i<3; i ++) this-> y[i] = y[i];
        this-> shape.setPointCount(ShapePoints);
        this-> shape.setFillColor(color);
        this-> shape.setRadius(size);
        this-> shape.setOrigin(size,size);
        NumberOfSpaceObjects++;

    }

    space_object::space_object(int windowSizeX, int windowSizeY){

        this->name = "";
        this->size = 10; // in px
        this->windowSizeX = windowSizeX; // in px
        this->windowSizeY = windowSizeY; // in px
        this->MaxSpeed = 60; // in px/s
        this-> x[0] = size; this-> x[1] = 0.; this-> x[2] = 0.;
        this-> y[0] = size; this-> y[1] = 0.; this-> y[2] = 0.;
        NumberOfSpaceObjects++;
    }
    space_object::~space_object(){ cout<< endl << name << " Destroyed...";}

//Class Shot that defines the object our spacecraft will create each time it fires
class Shot: public space_object {

	private:
		
        /* //Differential system that caracterizes mouvement
        void dynamics(int n, double t, double y[], double dy[]){
            //How to implment gravitational force betwen objects?
            dy[0] = y[1];
    		dy[1] = y[2];
        } */ 

        double ttl; // time to live of the shot

        double createdTime;

	public:

        // Remove a bullet if its TTL has expired
        bool Remove(){
            return (GetTickCount() - createdTime >= ttl);
        }

        //Shot Consructor ex: Bullet(Simple2D *r, Player &p, float sX, float sY, float sA);
        Shot(string name, int ShapePoints, Color color, double size, double ttl, int windowSizeX, int windowSizeY, double MaxSpeed , double x[], double y[],double phy):space_object(name, ShapePoints, color, size, windowSizeX, windowSizeY, MaxSpeed , x, y,phy){
            this-> ttl = ttl;
            this-> createdTime = GetTickCount();
        }

        // Delete a bullet
        ~Shot(){}

};

//Class that defines players it inherits methodes and variables from space object class
class ship: public space_object{

	private:
 		void dynamics(int n, double t, double y[], double dy[]){

    		dy[0] = y[1];
    		dy[1] = y[2] - 0.7*y[1]; // -y[1] is a non conservative force (it makes the control of the spacecraft easier)

		}

        //Description of shots fired (Misils or Bullets?)

        // Number of shots currently in use
        int shotsUsed;

        // Max number of allowed shots
        int maxShots;

        // Minimum time between each shot (ms)
        unsigned int shotCooldown;

        // Time last shot was fired
        unsigned int lastShotTime;

        bool firing;

	public:



        // Create a new shot and return a pointer to it
    	Shot *Fire();
        bool IsFiring(){return firing;};

        // Release one shot slot
	    void EndFire();

	    // Allow a new shot to be fired immediately if any slots free
	    void ResetShotCooldown();

        //Add control over the ship
        void GetInput(int sensibility);

        ship(string name, int ShapePoints, Color color, double size, int maxShots, int windowSizeX, int windowSizeY, double MaxSpeed, double x[], double y[], double phy);
        ship(int windowSizeX, int windowSizeY):space_object(windowSizeX,windowSizeY){};
        ~ship();
};
//Methods related to ship objects
    Shot* ship::Fire(){

        // Don't fire unless the cooldown period has expired
        if (GetTickCount() - lastShotTime >= shotCooldown)
        {
            // Don't fire if the maximum number of Shots are already on screen
            if (shotsUsed < maxShots)
            {
                double x[3]={GetVectorX()[0],100,0};
                double y[3]={GetVectorY()[0],100,0};
                // Make new Shot (change the sprintf for a string stream)
                Shot *shot = new Shot( "Boom... " + to_string(NumberOfSpaceObjects), 30, Color::Black, 10, 1e3, GetWindowSizeX(), GetWindowSizeY(),300, x, y, phy);

                // Last Shot fired now
                lastShotTime = GetTickCount();
                shotsUsed++;

                return shot;
            }
        }
        return NULL;
    }

    // Stop firing a bullet (called back when a Bullet object is destroyed)
    void ship::EndFire(){
        shotsUsed = max(shotsUsed - 1, 0);
    }

    // Reset cooldown (used when fire key is released)
    void ship::ResetShotCooldown(){
        lastShotTime = 0;
    }


    void ship::GetInput(int sensibility){

            if ( Keyboard::isKeyPressed(sf::Keyboard::Left) ) this->GetVectorX()[2] = -sensibility;
            else if ( Keyboard::isKeyPressed(sf::Keyboard::Right) ) this->GetVectorX()[2] = +sensibility;
            else if ( Keyboard::isKeyPressed(sf::Keyboard::Up) ) this->GetVectorY()[2] = -sensibility;
            else if ( Keyboard::isKeyPressed(sf::Keyboard::Down) ) this->GetVectorY()[2] = +sensibility;
            else if ( Keyboard::isKeyPressed(sf::Keyboard::Space) ) firing = true;
            else {
                this->GetVectorX()[2] = 0.;
                this->GetVectorY()[2] = 0.;
                ResetShotCooldown();
                firing = false;
            }

        }
    ship::ship(string name, int ShapePoints, Color color, double size, int maxShots, int windowSizeX, int windowSizeY, double MaxSpeed , double x[], double y[],double phy):space_object(name, ShapePoints, color, size, windowSizeX, windowSizeY, MaxSpeed , x, y, phy){
        this->maxShots = maxShots;
        this->firing = false;
    }
    ship::~ship(){}


int main(){

    int windowSizeX = 800, windowSizeY = 600;
	RenderWindow window(VideoMode(windowSizeX, windowSizeY), "Spacecraft Movement");
    window.setFramerateLimit(40);


	double rayon = 10; // rayon du cercle
    double x[]={rayon,0.,0.},y[]={rayon,0.,0.}; // position initiale du cercle x[0],y[0], vitesse x[1], y[1] et acceleration x[3], x
												// x[2] et y[2] Intensité des forces subies par le cercle exprimees dans la base canonique.
    int vmax = 100, maxShots = 10;
    double phy = 0;

    vector<Shot> bulletsFired;
    vector<ship> spacecraffts;

    ship p("player1", 3, Color::Green, rayon, maxShots, windowSizeX, windowSizeY, vmax, x, y, phy);
	spacecraffts.push_back(p);

    while (window.isOpen()){

        Event event;

        while (window.pollEvent(event)){
            if (event.type == Event::Closed) window.close();
          }

        //Sets the background
		window.clear(Color::Black);
        //clears the terminal
        system("clear");

		//Get the input from the arrows in the keyboard
		p.GetInput(127);

        bool isfirinig = false;
        if ( Keyboard::isKeyPressed(sf::Keyboard::Space) ) isfirinig=true;
        
        if (isfirinig){
            //double xBullet[3]= {p.GetVectorX()[0],0,0}, yBullet[3]= {p.GetVectorX()[0],0,0};
            Shot newBullet ( "Boom... ", 30, Color::Blue, 5, 1e3, p.GetWindowSizeX(), p.GetWindowSizeY(),300, p.GetVectorX(), p.GetVectorY(), phy);
            newBullet.SetSpeed(50.,50.);
            newBullet.SetForces(0.,0.);

            bulletsFired.push_back(newBullet);
            isfirinig = false;
            /* 
            To try XD
            bulletsFired.push_back(*bullet);
            for (int i =0 ; i < bulletsFired.size(); i++){
                window.draw( (*bullet).GetShape() );
                if((*bullet).Remove()) p.EndFire();
            } */
        }
        
        cout << "\n Bullets fired vector size:  " << bulletsFired.size()<<endl
             << "is firinig:    "<< isfirinig << endl;
        double sum = 0;
        for (int i = 0 ; i < bulletsFired.size();i++){
            //bulletsFired[i].SetForces(0.,0.);
            bulletsFired[i].UpdatePosition();
            window.draw(bulletsFired[i].GetShape());
            sum += p.distance(bulletsFired[i]);
            cout << "\nData of bullet:   " << i;
            bulletsFired[i].GetAll();
        }
        cout << "\nSum of bullets distace from ship: " << sum << endl;
		//Update position
		p.UpdatePosition();
 		
		//Check control
        p.GetAll();
        
        window.draw(p.GetShape());

        window.display();

    }

    p.~ship();
    return 0;
}

