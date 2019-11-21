#include <cstdlib>
#include <iostream>
#include<math.h>
#include <SFML/Graphics.hpp> // un seul include suffit pour avoir les trois parties essentielles
						     // de la SFML: graphics, window et system

using namespace std;
using namespace sf;

class space_object {

    private:

    	string name;
		double size;

		double x[3],y[3]; /*Canonical vectors, range 0 is the position of the object,
                            ranges above are the derivates in time
                            */
        //prototype differential system made to be rewriten

        double MaxSpeed;
        static int windowSizeX;
        static int windowSizeY;
        void dynamics(int n, double t, double y[], double dy[]);
        

    public:

        static int NumberOfSpaceObjects;

        string GetName(){ return name;}

        void SetName(string name){ this->name = name;}
        
        double GetSize(){ return size;}
        void SetSize(int size){ this->size = size;}

        void SetLimits(int windowSizeX, int windowSizeY,double MaxSpeed){

            this->windowSizeX = windowSizeX;
            this->windowSizeY = windowSizeY;
            this->MaxSpeed = MaxSpeed;

        }

        void ApplyLimits(){
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

        
        static int SetNumberOfSpaceObjects (int input){ NumberOfSpaceObjects = NumberOfSpaceObjects; }
        static int GetNumberOfSpaceObjects (){ return NumberOfSpaceObjects; }

        void SetAll(string name, double size, int windowSizeX, int windowSizeY, double MaxSpeed , double x[], double y[]);
        void GetAll();

        //Constructors
        space_object(string name, double size, int windowSizeX, int windowSizeY, double MaxSpeed, double x[], double y[]);
        space_object(int windowSizeX, int windowSizeY);

        //Destructor
        ~space_object(); // Frees up memory after objects are destroyed (To be defined - investigate)

};


//Instance of Static Variables

int space_object::windowSizeX = 800;
int space_object::windowSizeY = 600;
int space_object::NumberOfSpaceObjects = 0;


//Methods related to Space Objects

void space_object::dynamics(int n, double t, double y[], double dy[]){

    		dy[0] = y[1];
    		dy[1] = y[2];

		}


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

    space_object::space_object(string name, double size, int windowSizeX, int windowSizeY, double MaxSpeed , double x[], double y[]){

        this->name = name;
        this->size = size;
        this->windowSizeX = windowSizeX;
        this->windowSizeY = windowSizeY;
        this->MaxSpeed = MaxSpeed;
        for(int i=0; i<3; i ++) this-> x[i] = x[i];
        for(int i=0; i<3; i ++) this-> y[i] = y[i];
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

//Class that defines players it inherits methodes and variables from space object class
class ship: public space_object{

	private:
		
        void dynamics(int n, double t, double y[], double dy[]){

    		dy[0] = y[1];
    		dy[1] = y[2];

		}
        
	public:

        //Add control over the ship
        void GetInput(int sensibility){

            if ( Keyboard::isKeyPressed(sf::Keyboard::Left) ) this->GetVectorX()[2] = -sensibility;
            else if ( Keyboard::isKeyPressed(sf::Keyboard::Right) ) this->GetVectorX()[2] = +sensibility;
            else if ( Keyboard::isKeyPressed(sf::Keyboard::Up) ) this->GetVectorY()[2] = -sensibility;
            else if ( Keyboard::isKeyPressed(sf::Keyboard::Down) ) this->GetVectorY()[2] = +sensibility;
            else {
                this->GetVectorX()[2] = 0.;
                this->GetVectorY()[2] = 0.;
            }

        }

        void UpdatePosition(){ 
            rk4(3, 1., GetVectorX(), 1e-1);
            rk4(3, 1., GetVectorY(), 1e-1);
            ApplyLimits();
        }

        ship(string name, double size, int windowSizeX, int windowSizeY, double MaxSpeed, double x[], double y[]);
        ship(int windowSizeX, int windowSizeY):space_object(windowSizeX,windowSizeY){};
        ~ship();
};
    //Methods related to ship objects
    ship::ship(string name, double size, int windowSizeX, int windowSizeY, double MaxSpeed , double x[], double y[]):space_object(name, size, windowSizeX, windowSizeY, MaxSpeed , x, y){}
    //ship::ship(int windowSizeX, int windowSizeY):space_object(windowSizeX, windowSizeY){}
    ship::~ship(){}

//Function that will define distance betwen space objects 
double distance(space_object& A, space_object& B){

    double dx2 = pow((A.GetVectorX()[0]-B.GetVectorX()[0]), 2);
    double dy2 = pow((A.GetVectorY()[0]-B.GetVectorY()[0]), 2);

    return sqrt(dx2 + dy2);

}

class shot: public space_object{

	private:
		
        //Differential system that caracterizes mouvement
        void dynamics(int n, double t, double y[], double dy[]); //How to implment gravitational force betwen objects?

	public:

        void UpdatePosition(){ 
            rk4(3, 1., GetVectorX(), 1e-1);
            rk4(3, 1., GetVectorY(), 1e-1);
            ApplyLimits();
        }

};

int main(){

    int windowSizeX = 800, windowSizeY = 600;
	RenderWindow window(VideoMode(windowSizeX, windowSizeY), "Spacecraft Movement");
    window.setFramerateLimit(25);

    

	int rayon = 10; // rayon du cercle
    double x[]={rayon,0.,0.},y[]={rayon,0.,0.}; // position initiale du cercle x[0],y[0], vitesse x[1], y[1] et acceleration x[3], x
												// x[2] et y[2] Intensité des forces subies par le cercle exprimees dans la base canonique.
    int vmax = 100;
    ship p("player1",rayon,windowSizeX,windowSizeY,vmax,x,y);

    
	
	CircleShape cercle(rayon);
    cercle.setFillColor(Color::Blue);
    cercle.setOrigin(rayon,rayon);
    
    while (window.isOpen()){

        Event event;

        while (window.pollEvent(event)){
            if (event.type == Event::Closed) window.close();
          }

		//Get the input from the arrows in the keyboard
		p.GetInput(10);

		//Update position
		p.UpdatePosition();

		//Sets the limits of the framework
		p.SetLimits(windowSizeX,windowSizeY,vmax);

 		
		//Check control
        p.GetAll();

		//Sets the background
		window.clear(Color::White);
        cercle.setPosition(p.GetVectorX()[0],p.GetVectorY()[0]);
        window.draw(cercle);

        window.display();
    }

    p.~ship();
    return 0;
}

