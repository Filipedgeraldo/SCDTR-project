/* CONSENSUS ALGORITHM

Written Functions:
	- isFeasible: checks if the obtained duty-cycle for the node is feasible;
	- NormSq: calculates the squared norm of a vector (useful in intermediate calculations);
	- Optimization: runs the consensus algorithm and finds the optimal solutions for
					the duty-cycles of the three nodes
	- Evaluate_cost: calculates the total cost for a given solution

Important Variables:
	K = vector of coupling gains
	dist = external disturbance
	ref = lower bound for illuminance
	z = rho*d_av - cost - y (d_av = array of duty-cycle averages, y = array of lagrange multipliers, cost = array of costs)
	ID = node ID

*/

float Evaluate_cost(float* sol)
{
	float c = 0;
	for(int i = 0; i < 3; i++)
		c += cost[i]*sol[i] + y[i]*(sol[i] - d_av[i]) + rho*(pow(sol[i],2) - pow(d_av[i],2))/2; 
	
	return c;
}

float NormSq(float* vect)
{
	float res = 0;
	for(int i = 0; i < 3; i++)
		res += pow(vect[i],2);

	return res;
}

float* Optimization(){
	float d_u[3] = {0}, d_bL[3] = {0}, d_b1[3] = {0}, d_b0[3] = {0}, d_L1[3] = {0}, d_L0[3] = {0};
	float z = 0, cost_best = 100000, c = 0, m = 0;
	float* sol;
	
	for(int i = 0; i < 3; i++)
	{
		d_u[i] = z[i]/rho;
		d_bL[i] = z[i]/rho - (K[i]/NormSq(K))*(dist - ref + K[i]*z[i]/rho);
		d_b0[i] = z[i]/rho;
		d_b1[i] = z[i]/rho;
		m = NormSq(K) - K[ID]^2;
		d_L1[i] = z[i]/rho - K[i]*(dist - ref)/m + (1/(rho*m))*K[i]*(K[ID] + z[ID] - z[i]*K[i]);
		d_L0[i] = z[i]/rho - K[i]*(dist - ref)/m + (1/(rho*m))*K[i]*(K[ID] + z[ID] - z[i]*K[i]);
		
		if(i == ID)
		{
			d_b0[i] = 0;
			d_b1[i] = 1;
			d_L1[i] = 1;
			d_L0[i] = 0;
		}
	}
	
	if(isFeasible(d_u[ID])) //unconstrained
	{
		c = Evaluate_cost(d_u);
		if(c < cost_best)
			cost_best = c;
		return d_u;
	}
	else if(isFeasible(d_bL[ID])) //linear boundary
	{
		c = Evaluate_cost(d_bL);
		if(c < cost_best)
		{
			cost_best = c;
			sol = d_bL;
		}
	}
	else if(isFeasible(d_b0[ID])) //0 boundary
	{
		c = Evaluate_cost(d_b0);
		if(c < cost_best)
		{
			cost_best = c;
			sol = d_b0;
		}
	}
	else if(isFeasible(d_b1[ID])) //100 boundary
	{
		c = Evaluate_cost(d_b1);
		if(c < cost_best)
		{
			cost_best = c;
			sol = d_b1;
		}
	}
	else if(isFeasible(d_L0[ID])) //linear and 0 boundaries
	{
		c = Evaluate_cost(d_L0);
		if(c < cost_best)
		{
			cost_best = c;
			sol = d_L0;
		}
	}
	else if(isFeasible(d_L1[ID])) //linear and 100 boundaries
	{
		c = Evaluate_cost(d_L1);
		if(c < cost_best)
		{
			cost_best = c;
			sol = d_L1;
		}
	}
	return sol;
}

int isFeasible(duty){
	double tol = 0.001;
	if(duty < 0 + tol) return 0;
	else if(duty > 1 + tol) return 0;
	else if(duty*gain + dist < ref) return 0;
	
	return 1;
}