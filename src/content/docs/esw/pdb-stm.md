---
title: "PDB STM"
---
## Thermistor Calculations

**Context**

Here is a representation of the thermistor setup


**Problem**

The ADC for the temperature sensor takes in voltage drop from a thermistor. We need to convert this to the temperature. However, datasheets for different thermistors differ in how they report how a change in temperature affects the thermistor. For example, during 2022-23's development cycle, the thermistor's temperature coefficient was given in terms of millivolts/degree Celsius. Meanwhile, the 2023-24 thermistor had its temperature coefficient given in ohms/degree Celsius.


**Solution**

### Volt(s) per degree Celcius
Example: [2022-2023 thermistor: MCP9701A](https://www.digikey.com/en/products/detail/microchip-technology/MCP9701T-E-TT/1987445)

Since we know the relationship between volts and Celcius, we can just get the temperature directly from the ADC.
The general equation for converting ADC value ($val_{ADC}$) to voltage ($V_{ADC}$), given a certain source voltage ($V_{source}$) and ADC bits ($bits_{ADC}$) is:
$$V_{ADC} = val_{ADC} * V_{source} * \frac{1}{2^{bits_{ADC}}}$$

So, we need to get the parameters of the ADC. The STM boards should have a 12 bit ADC. The thermistor is also powered by a 3.3V source. This means the math to convert the ADC value to voltage will be:
$$V_{ADC} = val_{ADC} * 3.3 * \frac{1}{4096}$$

Once we have this, we can just calculate the difference between the ADC voltage and the voltage at a base temperature ($V_{base}$, given in datasheet). Then, we can use the temperature coefficient ($Co_{temp}$) to find out the temperature change that caused the change in voltage. In math terms:
$$Temp = \frac{V_{ADC} - V_{base}}{Co_{temp}} + temp_{base}$$

For the 2022-2023 development cycle, the base temperature was 0 degrees celcius, the voltage at that temperature was 0.4V, and the temperature coefficient was 0.0195V/degree Celcius. So, the equation becomes:
$$Temp = \frac{V_{ADC} - 0.4}{0.0195} + 0$$

### Ohm(s) per degree Celcius
Example: [2023-2024 themistor: TMP6431DECR](https://www.digikey.com/en/products/detail/texas-instruments/TMP6431DECR/11635694)

The calculations for this one are a bit more complicated than the Volts/Celcius section. But don't worry, it's fun! 😄 

The basic premise is the same. We have to convert the ADC value to Celcius. The ADC value is linked to the voltage of the thermistor. We then need to use that to find the resistance of the thermistor, and then find the temperature.

Finding the voltage from the ADC value is the same as the first section, or:
$$V_{ADC} = val_{ADC} * V_{source} * \frac{1}{2^{bits_{ADC}}}$$

And after we plug in the same values, we get:
$$V_{ADC} = val_{ADC} * 3.3 * \frac{1}{4096}$$

To get the resistance from the voltage, it might be easier to get the voltage from the resistance first, and then rearrange the equation. Since the output to the ADC is in parallel to the thermistor, the voltage drop over the themistor is the same as the voltage read by the ADC. From the diagram at the top of the page, we can see that the circuit forms a voltage divider. This means the voltage drop over the thermistor will be the voltage source times the thermistor resistance divided by the total resistance, or:
$$V_{therm} = V_{source}\frac{R_{therm}}{R_{therm}+10000}$$

After some rearranging to solve for the resistance of the thermistor, we have:
$$R_{therm} = \frac{\frac{10000*V_{therm}}{V_{source}}}{1-\frac{V_{therm}}{V_{source}}}$$

Which can also be written as
$$R_{therm} = \frac{10000*V_{therm}}{V_{source}-V_{therm}}$$

The source voltage is the same 3.3V, so we have:
$$R_{therm} = \frac{10000*V_{therm}}{3.3-V_{therm}}$$

Now, since we have to resistance of the thermistor, we can relate that to the temperature. The 2023-2024 thermistor had a temperature coefficient($Co_{temp}$) of 6400 ppm per degree Celcius, or 6400 microOhms per degree Celcius or $6.400*10^{-3}$ Ohms per degree Celcius. 

We also know the resistance at a base temperature($R_{base}$). In this case, the resistance at 25 degrees Celcius is 47000 Ohms. That means we can make the following equation:
$$Temp = \frac{R_{therm}-R_{base}}{Co_{temp}}+temp_{base}$$

And after plugging in the values we have:
$$Temp = \frac{R_{therm}-47000}{6.400*10^{-3}}+25$$

Now, we can plug the resistance of the thermistor into the equation and get the temperature in terms of the ADC value (as stated above, the voltage read by the ADC is the same as the voltage drop over the thermistor, so we can substitute $V_{ADC} = V_{therm}$):
$$Temp = \frac{10000*\frac{val_{ADC} * 3.3 * \frac{1}{4096}}{3.3-(val_{ADC} * 3.3 * \frac{1}{4096})}-47000}{6.400*10^{-3}}+25$$

In the code, this equation is broken down into the ADC Voltage, Thermistor Resistance, and Final Temperature for readability.


## ADC Current Calculations

**Context**
We have a PDB on the rover.

![ESW System 2024 drawio](https://github.com/umrover/mrover-ros/assets/71603173/13eb49e7-a9c3-4e05-b9bf-7684708c8dbd)

**Problem**


**Solution**