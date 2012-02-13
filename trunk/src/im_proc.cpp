#include "im_proc.hpp"

long long int factorial(int num)
{

    long long int result=1;
    for (int i=1; i<=num; ++i)
        result=result*=i;
    return result;

}

void getNextCombo(vector<unsigned int>& currentIndices, int r, int n)
{

    bool maxed = false;
    bool valid = true;

    //printf("%s << Entered function.\n", __FUNCTION__);

    // If no indices tested, use default (0, 1, 2 etc)
    if (currentIndices.size() == 0)
    {
        for (int i = 0; i < r; i++)
        {
            currentIndices.push_back(i);
        }
    }
    else
    {

        // Going back each digit
        int i = 0;

        while (valid && (i < r))
        {
            //printf("%s << i = %d / %d\n", __FUNCTION__, i, r);
            // If current index is about to go over its maximum...
            if (currentIndices.at(currentIndices.size()-i-1) > (n-2-i))
            {
                //printf("%s << digit #(%d) is valid; less than %d\n", __FUNCTION__, currentIndices.size()-i-1, n-2-i);
                i++;    // check out next index
            }
            else        // Otherwise, just increment it, fill in trailing digits and exit while loop
            {
                currentIndices.at(currentIndices.size()-i-1) = currentIndices.at(currentIndices.size()-i-1) + 1;
                for (int j = 0; j < i; j++)
                {
                    currentIndices.at(currentIndices.size()-i+j) = currentIndices.at(currentIndices.size()-i+j-1) + 1;
                }
                valid = false;
            }
        }


    }
}

double findEquivalentProbabilityScore(double* values, int quantity, double prob)
{

    int i = 0;
    vector<double> listVector;
    double min;
    int minIndex;

    // Push values into vector
    for (int j = 0; j < quantity; j++)
    {
        listVector.push_back(values[j]);
    }

    // Pop minimum values off vector until you've reached sufficient depth for the probability
    while (listVector.size() >= (unsigned int)(std::max(int(prob*quantity), 1)))
    {
        //printf("%s << listVector.size() = %d, prob*quantity = %d\n", __FUNCTION__, listVector.size(), int(prob*quantity));
        //cin.get();
        min = 9e99;

        for (unsigned int j = 0; j < listVector.size(); j++)
        {
            if (listVector.at(j) < min)
            {
                min = listVector.at(j);
                minIndex = j;
            }
        }

        listVector.erase(listVector.begin() + minIndex);

    }

    return min;

}
