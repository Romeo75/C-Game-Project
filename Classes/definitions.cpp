#include <cstdlib>
#include <iostream>
#include <math.h>
#include <vector>
#include<chrono>
#include <SFML/Graphics.hpp> // un seul include suffit pour avoir les trois parties essentielles
						     // de la SFML: graphics, window et system

using namespace std;
using namespace sf;

//Function for time reference it marks the begining of the counter
auto started = chrono::high_resolution_clock::now();

class space_object {

    private:

    	string name;

        
        double MaxSpeed;
		double sizeX,sizeY;
        static int windowSizeX;
        static int windowSizeY;

        virtual void dynamics(int n, double t, double y[], double dy[]){

    		dy[0] = y[1];
    		dy[1] = y[2];

		}

    public:
    
        double x[3],y[3]; /*Canonical vectors, range 0 is the position of the object,
                    ranges above are the derivates in time
                    */
                   
        /// TEXTURES \\\

        Sprite shape;
        Texture texture;
        vector<space_object> ObjectsInSpace;

        //Defined by partner for rotation mouvement rho and phy 
        double trust,phy; //provisonal

        string GetName(){ return name;}
        void SetName(string name){ this->name = name;}
        
        double GetSizeX(){ return sizeX;}
        double GetSizeY(){ return sizeY;}
        
        double GetMaxSpeed(){ return MaxSpeed;}

        void SetSize(int sizeX,int sizeY){
            this->sizeX = sizeX;
            this->sizeX = sizeX;
        }

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
            auto done = chrono::high_resolution_clock::now();
            return chrono::duration_cast<chrono::milliseconds>(done-started).count();
        }

        virtual void ApplyLimits(){
            /*
            This sets the limits of the space object
            */

            //Deffines Spacial limits
            if ( x[0] >= (windowSizeX-sizeX*0.5) ){
                x[1]=-x[1];
                x[0]=windowSizeX-sizeX*0.5;
            }

            if ( x[0] < sizeX*0.5 ) {
                x[1]=-x[1];
                x[0]=sizeX*0.5;
            } 
        
            if ( y[0] >= (windowSizeY-sizeY*0.5) ){
                y[1]=-y[1];
                y[0]=windowSizeY-sizeY*0.5;
            }

            if ( y[0] < sizeY*0.5 ) {
                y[1]=-y[1];
                y[0]=sizeY*0.5;
            }

            //Deffines Speed limits
            if ( x[1] > abs(MaxSpeed) ){
                x[1]=MaxSpeed;
            }

            if ( y[1] > abs(MaxSpeed) ){
                y[1]=MaxSpeed;
            }
            
            if ( x[1] < -abs(MaxSpeed) ){
                x[1]=-MaxSpeed;
            }

            if ( y[1] < -abs(MaxSpeed) ){
                y[1]=-MaxSpeed;
            }

        }

        double * GetVectorX(){ return x;}
        double * GetVectorY(){ return y;}

        double GetDirectionX() { return cos( phy*0.017453f );}
        double GetDirectionY() { return sin( phy*0.017453f );}

        void SetPosition(double xSpeed,double ySpeed){
            this->x[0] = xSpeed;
            this->y[0] = ySpeed;
        }

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
                { y[i] += dx*( d1[i] + 2*d2[i] + 2*d3[i] + d4[i] )/6 ; }
        }
        
        virtual void UpdatePosition(){
            
            rk4(3, 0., x, 1e-1);
            rk4(3, 0., y, 1e-1);
            ApplyLimits();
            shape.setPosition(x[0],y[0]);
            shape.setRotation(phy - 90);

        }


        void SetAll(string name, double sizeX, double sizeY, int windowSizeX, int windowSizeY, double MaxSpeed , double x[], double y[]);
        
        virtual void GetAll();

        //Constructors
        space_object(string name, int ShapePoints, Color color, string picture, double sizeX, double sizeY, int windowSizeX, int windowSizeY, double MaxSpeed, double x[], double y[], double phy);
        space_object(int windowSizeX, int windowSizeY);

        //Destructor
        ~space_object(); // Frees up memory after objects are destroyed (To be defined - investigate)

};

//Instance of Static Variables

    int space_object::windowSizeX = 800;
    int space_object::windowSizeY = 600;


//Methods related to Space

    void space_object::SetAll(string name, double sizeX, double sizeY, int windowSizeX, int windowSizeY, double MaxSpeed , double x[], double y[]){

        this->name = name;
        this->sizeX = sizeX;
        this->sizeY = sizeY;
        this->windowSizeX = windowSizeX;
        this->windowSizeY = windowSizeY;
        this->MaxSpeed = MaxSpeed;
        for(int i=0; i<3; i ++) this-> x[i] = x[i];
        for(int i=0; i<3; i ++) this-> y[i] = y[i];

    }

    void space_object::GetAll(){

        //Check control
        cout<< endl
            << "Number of Space Objects:    " << ObjectsInSpace.size() << endl
            << "Name: " << name << endl
            <<  "WindowSizeX: " << windowSizeX << endl
            <<  "WindowSizeY: " << windowSizeY << endl
            <<  "MaxSpeed:  "    << MaxSpeed << endl
            <<  "   Rotation:   " << phy << endl
            <<  "   Position: ("    << x[0]<< ","<<y[0]<<")"
            <<  "   Vitesse: ("     << x[1]<< ","<<y[1]<<")"
            <<  "   acceleration: ("<< x[2]<< ","<<y[2]<<")\n";
    }

    space_object::space_object(string name, int ShapePoints, Color color, string picture, double sizeX, double sizeY, int windowSizeX, int windowSizeY, double MaxSpeed , double x[], double y[], double phy){

        this-> name = name;
        this-> sizeX = sizeX;
        this-> sizeY = sizeY;
        this-> windowSizeX = windowSizeX;
        this-> windowSizeY = windowSizeY;
        this-> MaxSpeed = MaxSpeed;
        this-> phy = phy;
        for(int i=0; i<3; i ++) this-> x[i] = x[i];
        for(int i=0; i<3; i ++) this-> y[i] = y[i];
        this-> texture.loadFromFile(picture);
        this-> shape.setTextureRect(IntRect(0,0,sizeX,sizeY));
        this-> shape.setOrigin(sizeX/2.,sizeY/2.);
        this-> shape.setRotation(phy - 90);
        this-> shape.setTexture(texture);
        this-> ObjectsInSpace.push_back(*this);

        //-------For shapes objects--------\\\\
        //this-> shape.setRadius(size);
        //this-> shape.setPointCount(ShapePoints); For Shapes
        //this-> shape.setFillColor(color);

    }

    space_object::space_object(int windowSizeX, int windowSizeY){

        this->name = "";
        this->sizeX = 10; // in px
        this->sizeY = 10;
        this->windowSizeX = windowSizeX; // in px
        this->windowSizeY = windowSizeY; // in px
        this->MaxSpeed = 60; // in px/s
        this-> x[0] = sizeX; this-> x[1] = 0.; this-> x[2] = 0.;
        this-> y[0] = sizeY; this-> y[1] = 0.; this-> y[2] = 0.;
    }
    space_object::~space_object(){ cout<< endl << name << " Destroyed...";}

//Class Shot that defines the object our spacecraft will create each time it fires
class Shot: public space_object {

	private:

        double ttl; // time to live of the shot

        double externalForces; // ? Force to take into account in the differential system of mouvement

        double createdTime;

	public:

        /* Sprite shapeShot;
        Texture textureShot;

        void draw2(RenderWindow& window){
            window.draw(shapeShot);
        }
         */
        
        void UpdatePosition(){
            
            cout<< "\n........................BigLol........................"<<endl;
            rk4(3, 0., x, 1e-1);
            rk4(3, 0., y, 1e-1);
            ApplyLimits();
            shape.setPosition(x[0],y[0]);
            shape.setRotation(phy - 90);
        /* 
            shapeShot.setPosition(x[0],y[0]);
            shapeShot.setRotation(phy - 90);
         */    

        }    

        // Remove a bullet if its TTL has expired
        bool Remove(){
            return (GetTickCount() - createdTime >= ttl);
        }

        //Shot Consructor
        Shot(string name, int ShapePoints, Color color, string picture, double sizeX, double sizeY, double ttl, int windowSizeX, int windowSizeY, double MaxSpeed , double x[], double y[],double phy):space_object(name, ShapePoints, color, picture, sizeX, sizeY, windowSizeX, windowSizeY, MaxSpeed , x, y,phy){
            this-> ttl = ttl;
            this-> createdTime = GetTickCount();
            this-> ObjectsInSpace.push_back(*this);
            
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

        

    public:

        // Number of shots currently in use
        int shotsUsed;

        // Max number of allowed shots
        int maxShots;

        // Minimum time between each shot (ms)
        unsigned int shotCooldown;

        // Time last shot was fired
        unsigned int lastShotTime;

        bool firing;
	
        //Vectors of shots fired (Misils or Bullets?) And ships
        vector<Shot> ShotsInSpace;
        vector<ship> ShipsInSpace;
        
        void GetAll(){

            //Check control
            cout<< endl
                <<  "Number of Space Objects:    " << ObjectsInSpace.size() << endl
                <<  "Name: " << GetName() << endl
                <<  "WindowSizeX: " << GetWindowSizeX() << endl
                <<  "WindowSizeY: " << GetWindowSizeY() << endl
                <<  "MaxSpeed:  "    << GetMaxSpeed() << endl
                <<  "Shots fired vector size:  " << (ShotsInSpace.size()) << endl
                <<  "Time (in ms):  "<< GetTickCount() << endl
                <<  "   Rotation:   " << phy << endl
                <<  "   Position: ("    << GetVectorX()[0]<< ","<<GetVectorY()[0]<<")"
                <<  "   Vitesse: ("     << GetVectorX()[1]<< ","<<GetVectorY()[1]<<")"
                <<  "   acceleration: ("<< GetVectorX()[2]<< ","<<GetVectorY()[2]<<")\n";
        }
        
        // Create a new shot and ensures that it doesn't goes like crazy
    	void Fire();

        // Release one shot slot
	    void EndFire();

	    // Allow a new shot to be fired immediately if any slots free
	    void ResetShotCooldown();

        //Add control over the ship
        void GetInput(int sensibility);

        ship(string name, int ShapePoints, Color color, string picture, double sizeX,double sizeY, int maxShots, int windowSizeX, int windowSizeY, double MaxSpeed, double x[], double y[], double phy);
        ship(int windowSizeX, int windowSizeY):space_object(windowSizeX,windowSizeY){};
        ~ship();
};
//Methods related to ship objects

    void ship::Fire(){

            // Don't fire unless the cooldown period has expired
            if ((GetTickCount() - lastShotTime) >= shotCooldown)
            {
                
                // Don't fire if the maximum number of Shots are already on screen
                if (shotsUsed < maxShots)
                {

                    double x[]={GetVectorX()[0], GetDirectionX()*100, 0};
                    double y[]={GetVectorY()[0], GetDirectionY()*100, 0};
                    
                    // Makes new Shot
                    Shot shot( "Boom... ", 30, Color::Black, "spaceMissil.png", 20, 35, 3e3, GetWindowSizeX(), GetWindowSizeY(), GetMaxSpeed(), x, y, phy + 180);

                    /* //Prepares the vector
                    shot.SetPosition( GetVectorX()[0], GetVectorY()[0] );
                    shot.SetSpeed(GetDirectionX()*100,GetDirectionY()*100);
                    shot.SetForces(0,0); */
                    
                    //Stores it in a vector
                    (this->ShotsInSpace).push_back(shot);
                    
                    // Last Shot fired now!!!!
                    lastShotTime = GetTickCount();
                    shotsUsed++;

                }
            }
            
        }

    
    /*
        Stop firing a bullet (called back when a Bullet object is destroyed)
    */
    void ship::EndFire(){
        shotsUsed = max(shotsUsed - 1, 0);
    }

    // Reset cooldown (used when fire key is released)
    void ship::ResetShotCooldown(){
        lastShotTime = 0;
    }


    void ship::GetInput(int sensibility){

            if ( Keyboard::isKeyPressed(sf::Keyboard::Left) )  {        this->phy   += -sensibility;
                if ( Keyboard::isKeyPressed(sf::Keyboard::Up) )         this->trust += sensibility;
                else if ( Keyboard::isKeyPressed(sf::Keyboard::Down) )  this->trust += -sensibility;    
            }

            else if ( Keyboard::isKeyPressed(sf::Keyboard::Right) ) {   this->phy   += +sensibility;
                if ( Keyboard::isKeyPressed(sf::Keyboard::Up) )         this->trust += sensibility;
                else if ( Keyboard::isKeyPressed(sf::Keyboard::Down) )  this->trust += -sensibility;
            }
            else if ( Keyboard::isKeyPressed(sf::Keyboard::Up) )        this->trust += sensibility;
            else if ( Keyboard::isKeyPressed(sf::Keyboard::Down) )      this->trust += -sensibility;

            else if ( Keyboard::isKeyPressed(sf::Keyboard::Space) ) firing = true;
            else {
            

                trust = 0.;
                this->GetVectorX()[2] = 0.;
                this->GetVectorY()[2] = 0.;
                ResetShotCooldown();
                firing = false;
            }

            GetVectorX()[2] = trust*GetDirectionX();
            GetVectorY()[2] = trust*GetDirectionY();

        }
    ship::ship(string name, int ShapePoints, Color color, string picture, double sizeX, double sizeY, int maxShots, int windowSizeX, int windowSizeY, double MaxSpeed , double x[], double y[],double phy):space_object(name, ShapePoints, color, picture, sizeX, sizeY, windowSizeX, windowSizeY, MaxSpeed , x, y, phy){
        this->maxShots = maxShots;
        this->shotsUsed = 0;
        this->lastShotTime = GetTickCount();
        this->firing = false;
        this->shotCooldown = 1000;
        this->ShipsInSpace.push_back(*this);
    }
    ship::~ship(){}


int main(){

    int windowSizeX = 800, windowSizeY = 600;
	RenderWindow window(VideoMode(windowSizeX, windowSizeY), "Spacecraft Movement");
    wi  ndow.setFramerateLimit(40);

    /// TEXTURES Of the Background\\\
    
    Texture t1;
    t1.loadFromFile("blue.png");
    t1.setRepeated(true);
    Sprite sFond(t1,IntRect(0,0,windowSizeX,windowSizeY));

	double sizeX = 100, sizeY = 94; // dimensions of the ship
    double x[]={sizeX+100,0.,0.},y[]={sizeY+100,0.,0.}; // position initiale du cercle x[0],y[0], vitesse x[1], y[1] et acceleration x[3], x
												// x[2] et y[2] Intensité des forces subies par le cercle exprimees dans la base canonique.
    int vmax = 100, maxShots = 10;
    double phy = 0;

    //vector<ship> spacecraffts;


    ship p("player1", 3, Color::Green, "spaceShip.png", sizeX, sizeY, maxShots, windowSizeX, windowSizeY, vmax, x, y, phy);
	//spacecraffts.push_back(p);

    while (window.isOpen()){

        Event event;

        while (window.pollEvent(event)){
            if (event.type == Event::Closed) window.close();
          }

        //Sets the background
		window.clear(Color::Black);
        //clears the terminal
        system("clear");
        
        window.draw(sFond);        

		//Get the input from the arrows in the keyboard
		p.GetInput(3);


        if ( p.firing ){
            
            cout<< endl << p.GetName() + "Fireeee!!!!!!!";
            p.Fire();
            p.ShotsInSpace[0].GetAll();

         }
       
        /* 
        //LOOOOOOOOOOOOOOOOOOOL
        x[2]=10;y[2]=10;
        Shot shot( "Boom... ", 30, Color::Black, "spaceMissil.png", 20, 35, 3e3, p.GetWindowSizeX(), p.GetWindowSizeY(), p.GetMaxSpeed(), x, y, p.phy);
        shot.UpdatePosition();
        shot.GetAll();
        shot.draw(window);
        */
        
        double sum = 0; //Test to see if it is possible to create the fonction force() using this method
        for (int i = 0 ; i < (p.ShotsInSpace).size(); i++){
            
            ( p.ShotsInSpace[i] ).UpdatePosition();
            (p.ShotsInSpace[i]).texture.loadFromFile("spaceMissil.png");
            (p.ShotsInSpace[i]).shape.setTexture((p.ShotsInSpace[i]).texture);
            (p.ShotsInSpace[i]).draw(window);
            (p.ShotsInSpace[i]).GetAll();

            sum += p.distance((p.ShotsInSpace)[i]);

            if( ((p.ShotsInSpace).front()).Remove() ) {
                p.EndFire();
                (p.ShotsInSpace).erase((p.ShotsInSpace).begin());
            }
        }
        cout << "\nSum of bullets distace from ship: " << sum << endl; 
        
		
        //Update position of all the elements related to the player p
		p.UpdatePosition();
 		
		//Check control
        p.GetAll();

        window.draw(p.shape);
        window.display();

    }

    p.~ship();
    return 0;
}
