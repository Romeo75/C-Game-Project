// ---------- INTRO TO CLASSES & INHERITANCE ----------

#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>
#include <sys/types.h>
#include <dirent.h>
#include <sys/stat.h>

class Animal{
    
    // Private variables are only available to methods 
    // in the class
private:
    std::string name;
    double height;
    double weight;
    
    // Static variables share the same value for all
    // objects of the Animal class
    static int numOfAnimals;

    // Public fields and methods can be accessed by
    // anything with access to the object
public:
    std::string GetName(){return name;}
    
    // Because we don't know what an objects name is
    // because the class is created before the object
    // we can refer to all objects using this
    // The pointer operator is used to access an 
    // objects fields and methods
    void SetName(std::string name){this->name = name;}
    double GetHeight(){return height;}
    void SetHeight(double height){this->height = height;}
    double GetWeight(){return weight;}
    void SetWeight(double weight){this->weight = weight;}
    
    // You can declare function prototypes
    void SetAll(std::string, double, double);
    
    // A constructor is called each time an object is created
    Animal(std::string, double, double);
    
    // Create an overloaded constructor for when no data is passed
    Animal();
    
    // A deconstructor is called automatically when an object
    // is deleted or is no longer used
    // The default is fine, but you should create custom ones
    // when you must release memory, or resources
    ~Animal();
    
    // Static methods can only access static fields
    static int GetNumOfAnimals(){return numOfAnimals;}
    
    // Created to be overwritten
    void ToString();
    
};

// Refer to class fields and methods with ::
int Animal::numOfAnimals = 0;

// Define the prototype method
void Animal::SetAll(std::string name, double height,
        double weight){
    this->name = name;
    this->weight = weight;
    this->height = height;
    Animal::numOfAnimals++;
}

// Define the constructor
Animal::Animal(std::string name, double height,
        double weight){
    this->name = name;
    this->weight = weight;
    this->height = height;
    Animal::numOfAnimals++;
}

Animal::Animal(){
    this->name = "";
    this->weight = 0;
    this->height = 0;
    Animal::numOfAnimals++;
}

Animal::~Animal(){
    std::cout << "Animal " << this -> name << 
            " destroyed\n";
}

void Animal::ToString(){
    std::cout << this -> name << " is " << 
            this -> height <<
            " cms tall and " << this -> weight <<
            " kgs in weight\n";
}

// Through inheritance a class inherits all fields and methods
// defined by the super, or inherited from class
class Dog: public Animal{
private:
    std::string sound = "Wooof";
public:
    // You can access to the private field name
    // by calling GetName()
    void MakeSound(){ 
        std::cout << "The dog " << 
                this->GetName() << " says " << 
                this->sound << "\n";
    }
    
    // The Dogs constructor
    Dog(std::string, double, double, std::string);
    
    // The default constructor calls Animals default
    // constructor
    Dog(): Animal(){};
    
    // Overwrite ToString
    void ToString();
    
};

// Calls the superclasses constructor to handle
// initalization
Dog::Dog(std::string name, double height, 
double weight, std::string sound) :
Animal(name, height, weight){
    this -> sound = sound;
}

// Overwrite ToString
void Dog::ToString(){
    // Because the attributes were private in Animal they must be retrieved 
    // by called the get methods
    std::cout << this -> GetName() << " is " << this -> GetHeight() << 
            " cms tall, " << this -> GetWeight() << 
            " kgs in weight and says " << this -> sound << "\n";
}
 
int main()
{
    // Create object without setting values in constructor
    Animal fred;
    fred.SetHeight(33);
    fred.SetWeight(10);
    fred.SetName("Fred");
    // Get the values for the Animal
    fred.ToString();
    
    fred.SetAll("Fred", 34, 12);
    
    fred.ToString();
    
    // Setting values with constructor
    Animal tom("Tom", 36, 15);
    tom.ToString();
    
    // Demonstrate inherited Dog class
    Dog spot("Spot", 38, 16, "Wooof");
    
    // See different output from overwritten ToString()
    spot.ToString();
    
    // Call static methods by using the class name to
    // show the total Animals created
    std::cout << "Number of Animals " << 
            Animal::GetNumOfAnimals() << "\n";
    
    return 0;
}

// ---------- END INTRO TO CLASSES & INHERITANCE ----------

// ---------- PROBLEM WARRIORS BATTLE TILL DEATH ----------
// Make this
/*
Thor attacks Hulk and deals 12 damage
Hulk is down to 28 health
Hulk attacks Thor and deals 3 damage
Thor is down to 37 health
Thor attacks Hulk and deals 14 damage
Hulk is down to 14 health
Hulk attacks Thor and deals 0 damage
Thor is down to 37 health
Thor attacks Hulk and deals 14 damage
Hulk is down to 0 health
Hulk has Died and Thor is Victorious
Game Over
*/

#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>
#include <ctime>
#include <numeric>
#include <math.h>

class Warrior{
private:
    int attkMax;
    int blockMax;
public:
    std::string name;
    int health;
    Warrior(std::string name, int health,
            int attkMax, int blockMax){
        this->name = name;
        this->health = health;
        this->attkMax = attkMax;
        this->blockMax = blockMax;
    }
    
    // The attack and block amount will be random
    int Attack(){
        return std::rand() % this->attkMax;
    }
    int Block(){
        return std::rand() % this->blockMax;
    }
};

class Battle{
public:
    // We pass warriors into the function by reference so we can
    // track continued damage to each
    
    // We continue to loop having each warrior take turns attacking
    // until a warriors health < 0
    static void StartFight(Warrior& warrior1, Warrior& warrior2){
        while(true){
            if(Battle::GetAttackResult(warrior1, warrior2).compare("Game Over") == 0){
                std::cout << "Game Over\n";
                break;
            }
            if(Battle::GetAttackResult(warrior2, warrior1).compare("Game Over") == 0){
                std::cout << "Game Over\n";
                break;
            }
        }
    }
    
    static std::string GetAttackResult(Warrior& warriorA, 
    Warrior& warriorB){
        // Get random attack & block amounts and calculate damage
        int warriorAAttkAmt = warriorA.Attack();
        int warriorBBlockAmt = warriorB.Block();
        int damage2WarriorB = ceil(warriorAAttkAmt - warriorBBlockAmt);
        
        // Change health total if > 0 and output changes
        damage2WarriorB = (damage2WarriorB <= 0) ? 0 : damage2WarriorB;
        warriorB.health = warriorB.health - damage2WarriorB;
        printf("%s attacks %s and deals %d damage\n", 
                warriorA.name.c_str(), warriorB.name.c_str(),
                damage2WarriorB);
        printf("%s is down to %d health\n", warriorB.name.c_str(),
                warriorB.health);
        
        // Once health < 0 end game by passing back Game Over
        if(warriorB.health <= 0){
            printf("%s has Died and %s is Victorious\n",
                    warriorB.name.c_str(), warriorA.name.c_str());
            return "Game Over";
        } else {
            return "Fight Again";
        }
    }
};
 
int main()
{
    // Seed the random number generator
    srand(time(NULL));
    
    // Create warriors
    Warrior tom("Thor", 100, 35, 15);
    Warrior sam("Hulk", 125, 25, 10);
    
    // Create battle and have it run on its own till completion
    Battle::StartFight(tom, sam);
    
    return 0;
}

// ---------- END PROBLEM WARRIORS BATTLE TILL DEATH ----------