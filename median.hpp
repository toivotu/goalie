#ifndef MEDIA_HPP
#define MEDIA_HPP

class Median {
public:
    Median(unsigned int _numSamples) : numSamples(_numSamples)
    {

    }

    float Evaluate(float value)
    {
        sum += value;
        values.push_back(value);

        if (values.size() > numSamples) {
            sum -= values.front();
            values.pop_front();
        }

        return sum / values.size();
    }

private:
    std::list<int> values;
    float sum;
    unsigned int numSamples;

};

#endif
