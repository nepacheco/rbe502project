function [D,C,G] = generate_full_model_v2(l,l_dot)
c = 0.288;
k = 2.4;
m = 1;

D = -8.364e-45*m*(7.462e+40*l*sin(0.2537*l) - 2.745e+43*cos(0.07248*l) - 8.236e+42*cos(0.145*l) - 2.745e+42*cos(0.1812*l) - 1.647e+43*cos(0.1087*l) - 4.118e+43*cos(0.03624*l) + 1.467e+42*l*sin(0.03624*l) + 1.94e+42*l*sin(0.07248*l) + 1.269e+42*l*sin(0.145*l) + 4.975e+40*l*sin(0.2899*l) + 2.487e+40*l*sin(0.3262*l) + 5.721e+41*l*sin(0.1812*l) + 1.716e+42*l*sin(0.1087*l) + 7.462e+40*l*sin(0.2175*l) + 8.789e+39*l^2*cos(0.2537*l) - 3.502e+41*l^2*cos(0.03624*l) - 2.163e+41*l^2*cos(0.07248*l) - 4.462e+40*l^2*cos(0.145*l) + 8.113e+39*l^2*cos(0.2899*l) + 4.958e+39*l^2*cos(0.3262*l) - 5.859e+39*l^2*cos(0.1812*l) - 1.158e+41*l^2*cos(0.1087*l) + 6.761e+39*l^2*cos(0.2175*l) - 2.531e+41*l^2 - 2.882e+43);
C = 0.030164*c*l_dot - 8.364e-45*l_dot*m*(7.4618e+40*l_dot*sin(0.25369*l) - 5.0616e+41*l*l_dot + 2.9599e+42*l_dot*sin(0.036242*l) + 3.9299e+42*l_dot*sin(0.072484*l) + 2.4624e+42*l_dot*sin(0.14497*l) + 4.9745e+40*l_dot*sin(0.28994*l) + 2.4873e+40*l_dot*sin(0.32618*l) + 1.0695e+42*l_dot*sin(0.18121*l) + 3.5071e+42*l_dot*sin(0.10873*l) + 7.4618e+40*l_dot*sin(0.21745*l) - 2.2297e+39*l^2*l_dot*sin(0.25369*l) + 1.2692e+40*l^2*l_dot*sin(0.036242*l) + 1.5682e+40*l^2*l_dot*sin(0.072484*l) + 6.4687e+39*l^2*l_dot*sin(0.14497*l) - 2.3522e+39*l^2*l_dot*sin(0.28994*l) - 1.6172e+39*l^2*l_dot*sin(0.32618*l) + 1.0618e+39*l^2*l_dot*sin(0.18121*l) + 1.2594e+40*l^2*l_dot*sin(0.10873*l) - 1.4702e+39*l^2*l_dot*sin(0.21745*l) + 3.6508e+40*l*l_dot*cos(0.25369*l) - 6.4723e+41*l*l_dot*cos(0.036242*l) - 2.9207e+41*l*l_dot*cos(0.072484*l) + 9.4651e+40*l*l_dot*cos(0.14497*l) + 3.0649e+40*l*l_dot*cos(0.28994*l) + 1.8029e+40*l*l_dot*cos(0.32618*l) + 9.1947e+40*l*l_dot*cos(0.18121*l) - 4.5072e+40*l*l_dot*cos(0.10873*l) + 2.9748e+40*l*l_dot*cos(0.21745*l)) - 1.5144e-64*l_dot^2*m*(2.0605e+60*sin(0.25369*l) - 1.3977e+61*l + 8.1734e+61*sin(0.036242*l) + 1.0852e+62*sin(0.072484*l) + 6.7997e+61*sin(0.14497*l) + 1.3737e+60*sin(0.28994*l) + 6.8684e+59*sin(0.32618*l) + 2.9534e+61*sin(0.18121*l) + 9.6845e+61*sin(0.10873*l) + 2.0605e+60*sin(0.21745*l) + 1.0082e+60*l*cos(0.25369*l) - 1.7873e+61*l*cos(0.036242*l) - 8.0652e+60*l*cos(0.072484*l) + 2.6137e+60*l*cos(0.14497*l) + 8.4635e+59*l*cos(0.28994*l) + 4.9785e+59*l*cos(0.32618*l) + 2.539e+60*l*cos(0.18121*l) - 1.2446e+60*l*cos(0.10873*l) + 8.2146e+59*l*cos(0.21745*l) - 6.1573e+58*l^2*sin(0.25369*l) + 3.5049e+59*l^2*sin(0.036242*l) + 4.3304e+59*l^2*sin(0.072484*l) + 1.7863e+59*l^2*sin(0.14497*l) - 6.4956e+58*l^2*sin(0.28994*l) - 4.4657e+58*l^2*sin(0.32618*l) + 2.932e+58*l^2*sin(0.18121*l) + 3.4778e+59*l^2*sin(0.10873*l) - 4.0597e+58*l^2*sin(0.21745*l));
G = 0.047553*k*l - 9.8*m*(0.32144*cos(0.12685*l) + 0.64288*cos(0.018121*l) + 0.21429*cos(0.16309*l) + 0.42858*cos(0.090605*l) + 0.10715*cos(0.19933*l) + 0.53573*cos(0.054363*l) - 0.040774*l*sin(0.12685*l) - 0.034949*l*sin(0.16309*l) - 0.038832*l*sin(0.090605*l) - 0.021358*l*sin(0.19933*l) - 0.029124*l*sin(0.054363*l) - 0.10873*sin(0.018121*l)*(0.10715*l - 5.0));
end