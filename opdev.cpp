#include<iostream>
#include<map>
#include<set>
#include<array>
#include<vector>
#include<string>
#include<sstream>
#include<memory>
#include<iomanip>
#include<algorithm>
#include "gurobi_c++.h"
using namespace std;

class operation{
private:
        int _id;
        int _time;
public:
        const int &time(){return _time;}
        const int &id(){return _id;}
        operation(int given_id, int given_time)
        :_id(given_id), _time(given_time){}
};

class device{
private:
        int _id;
public:
        const int &id(){return _id;}
        device(int given_id):_id(given_id){}
};
class variable{
public:
        int id;
        int type;
        double upbound;
        double lobound;
        bool bool_value;
        int int_value;
        double double_value;
        set<int> constr_involved;
        map<int,double> constr_coeff;

};

int objcons;
int sum_constr;
map<int,int> constr_sense_map;
map<int,double> constr_rightside_map;
map<int,set<shared_ptr<variable>>> constr_variable_map;
vector<shared_ptr<variable>> vvar;
map<int,map<int,shared_ptr<variable>>> device_map;
shared_ptr<variable> Finaltime;
map<int,shared_ptr<variable>> device_working;
vector<shared_ptr<operation>> operations;
vector<shared_ptr<device>> devices;
/*map<int,set<variable*>> constr_variable_map;
vector<variable*> vvar;
map<int,map<int,variable*>> device_map;
variable* Finaltime;
map<int,variable*> device_working;
vector<operation*> operations;
vector<device*> devices;*/

/*device_map[i][j]=1 means the j-th operation happens in device i;*/
void set_device_working_map(){
        for(int i=1;i<=devices.size();i++){
                for(int j=1;j<=operations.size();j++){
                        device_map[i][j]=make_shared<variable>();
                        device_map[i][j]->id=vvar.size();
                        device_map[i][j]->type=0;
                        device_map[i][j]->upbound=1;
                        device_map[i][j]->lobound=0;
                        vvar.push_back(device_map[i][j]);
                }
        }
}

/*each operation can only be executed in one device;*/
void set_constraint_group1(){
        for(int j=1;j<=operations.size();j++){
                sum_constr++;
                constr_sense_map[sum_constr]=0;
                constr_rightside_map[sum_constr]=1;
                for(int i=1;i<=device_map.size();i++){
                        constr_variable_map[sum_constr].insert(device_map[i][j]);
                        device_map[i][j]->constr_involved.insert(sum_constr);
                        device_map[i][j]->constr_coeff[sum_constr]=1;
                }
        }
}
void set_device_workingtime(){
        for(int i=1;i<=devices.size();i++){
                device_working[i]=make_shared<variable>();
                device_working[i]->id=vvar.size();
                device_working[i]->type=1;
                device_working[i]->upbound=1000;
                device_working[i]->lobound=0;
                vvar.push_back(device_working[i]);
        }
}

/*device working time equal the sum of d[i][j]*operation->time*/
void set_constraint_group2(){
        for(int i=1;i<=devices.size();i++){
                sum_constr++;
                constr_sense_map[sum_constr]=0;
                constr_rightside_map[sum_constr]=0;
                for(int j=1;j<=operations.size();j++){
                        constr_variable_map[sum_constr].insert(device_working[j]);
                        device_working[j]->constr_involved.insert(sum_constr);
                        device_working[j]->constr_coeff[sum_constr]=-1;
                        constr_variable_map[sum_constr].insert(device_map[i][j]);
                        device_map[i][j]->constr_involved.insert(sum_constr);
                        device_map[i][j]->constr_coeff[sum_constr]=operations[j-1]->time();
                }
        }
}

/*set the final working time of the max of three devices*/

void set_final_time(){
        Finaltime=make_shared<variable>();
        Finaltime->id=vvar.size();
        Finaltime->type=1;
        Finaltime->upbound=1000;
        Finaltime->lobound=0;
        vvar.push_back(Finaltime);
}

/*Finaltime>=every device working time*/
void set_constraint_group3(){
        for(int i=1;i<=devices.size();i++){
                sum_constr++;
                constr_sense_map[sum_constr]=1;
                constr_rightside_map[sum_constr]=0;
                constr_variable_map[sum_constr].insert(Finaltime);
                Finaltime->constr_involved.insert(sum_constr);
                Finaltime->constr_coeff[sum_constr]=1;
                constr_variable_map[sum_constr].insert(device_working[i]);
                device_working[i]->constr_involved.insert(sum_constr);
                device_working[i]->constr_coeff[sum_constr]=-1;
        }
	sum_constr++;
	constr_variable_map[sum_constr].insert(Finaltime);
	Finaltime->constr_involved.insert(sum_constr);
	Finaltime->constr_coeff[sum_constr]=1;
}

void funcGurobi(double time, double absgap, int idisplay)
{
        GRBEnv env = GRBEnv();
        GRBModel model = GRBModel(env);
        model.getEnv().set(GRB_DoubleParam_TimeLimit, 102); // case 900
//      model.getEnv().set(GRB_DoubleParam_MIPGapAbs, 0); // case 0.02
//      model.getEnv().set(GRB_DoubleParam_Heuristics, 1); // case none
//      model.getEnv().set(GRB_DoubleParam_ImproveStartGap, 0.02); // case 0.02
        model.getEnv().set(GRB_IntParam_OutputFlag, idisplay);
        map<shared_ptr<variable>,string> mapvs;
        for(int i=0;i<vvar.size();i++)
        {
                ostringstream convi;
                convi<<i;
                mapvs[vvar[i]]="x"+convi.str();
        }
//	vector<shared_ptr<GRBVar>> x;
	GRBVar *x=new GRBVar [vvar.size()+1];
//	auto x=make_shared<GRBVar>(vvar.size()+1);	
        for(int i=0;i<vvar.size();i++)
        {
                if(vvar[i]->type==0)
                x[i]=model.addVar(vvar[i]->lobound,vvar[i]->upbound,0.0,GRB_BINARY,mapvs[vvar[i]]);
                else if(vvar[i]->type==1)
                x[i]=model.addVar(vvar[i]->lobound,vvar[i]->upbound,0.0,GRB_INTEGER,mapvs[vvar[i]]);
                else if(vvar[i]->type==2)
                x[i]=model.addVar(vvar[i]->lobound,vvar[i]->upbound,0.0,GRB_CONTINUOUS,mapvs[vvar[i]]);
        }
        model.update();

        for(int i=1;i<=sum_constr;i++)
        if(constr_variable_map[i].size()!=0) // cons with 0 var is eliminated
        {
                ostringstream convi;
                convi<<i;
                GRBLinExpr expr;

                if(i!=objcons)
                {
                        for(auto setit:constr_variable_map[i])
			//for(set<variable*>::iterator setit=constr_variable_map[i].begin();setit!=constr_variable_map[i].end();setit++)
                        {
                                expr+=x[setit->id]*(setit->constr_coeff[i]);
                                //expr+=x[(*setit)->index]*((*setit)->constr_coeff[i]);
                        }
                        string name='c'+convi.str();
                        if(constr_sense_map[i]==1)
                                model.addConstr(expr,GRB_GREATER_EQUAL,constr_rightside_map[i],name);
                        else if(constr_sense_map[i]==-1)
                                model.addConstr(expr,GRB_LESS_EQUAL,constr_rightside_map[i],name);
                        else if(constr_sense_map[i]==0)
                                model.addConstr(expr,GRB_EQUAL,constr_rightside_map[i],name);
                }
                else
                {
			for(auto setit:constr_variable_map[i]){
//                        for(set<variable*>::iterator setit=constr_variable_map[i].begin();setit!=constr_variable_map[i].end();setit++)
                                expr+=x[setit->id]*(setit->constr_coeff[i]);
                //      model.setObjective(expr,GRB_MAXIMIZE);
                        	model.setObjective(expr,GRB_MINIMIZE);
                }
        }
//      mycallback cb(time, absgap);
//      model.setCallback(&cb);
        model.optimize();

        while(model.get(GRB_IntAttr_SolCount)==0)
        {
                time+=5;
                model.getEnv().set(GRB_DoubleParam_TimeLimit, time); // case 900
                model.optimize();
        }
        for(int i=0;i<vvar.size();i++)
        {
                if(vvar[i]->type==0||vvar[i]->type==1)
                {
                        if(x[i].get(GRB_DoubleAttr_X)-(int)x[i].get(GRB_DoubleAttr_X)<0.5)
                                vvar[i]->int_value=(int)x[i].get(GRB_DoubleAttr_X);
                        else
                                vvar[i]->int_value=(int)x[i].get(GRB_DoubleAttr_X)+1;
                }
                else if(vvar[i]->type==2)
                        vvar[i]->double_value=x[i].get(GRB_DoubleAttr_X);
                else
                        cout<<"new type"<<endl;
        }
}
}

int main(int argc, char* argv[])
{
        operations.push_back(make_shared<operation>(1, 10));
        operations.push_back(make_shared<operation>(2, 20));
        operations.push_back(make_shared<operation>(3, 30));
        operations.push_back(make_shared<operation>(4, 40));
        operations.push_back(make_shared<operation>(5, 50));
        operations.push_back(make_shared<operation>(6, 60));
        for(int j=1;j<=3;j++)
                devices.push_back(make_shared<device>(j));
//        if(argc=1){
	set_device_working_map();
	set_constraint_group1();
	set_device_workingtime();
	set_constraint_group2();
	set_final_time();
	set_constraint_group3();
	cout<<"wwwwwwwwwwww2"<<endl;
	objcons=sum_constr;
	funcGurobi(30,0,1);
//	}
}













