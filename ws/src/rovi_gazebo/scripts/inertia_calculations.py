import numpy as np



coffe_volume = 0.001424
coffe_can_density = 1000

coffecanI = np.array([63688.339844, 0.000015, -648.260803, 
                     0.000015, 64073.300781, -0.003742, 
                    -648.260803, -0.003742, 18547.197266]
)



coffe_scale_factor = 100

coffecanI = coffe_can_density*coffecanI/(coffe_scale_factor**5)
print("CoffeCan Inertia...")
print(coffecanI)
coffecanI = coffecanI.reshape(3,3)




milk_volume = 0.001011
milk_density = 1000
milkI = np.array([38363.679688, 0.000002, 0.000061,
                 0.000002, 38255.585938, -9.417378,
                 0.000061, -9.417378, 8372.243164])

milkI_scale_factor = 100
milkI = (milk_density*milkI/(milkI_scale_factor**5)).reshape(3,3)



mug_volume = 0.000255
mug_density = 1000
mugI = np.array([4689.132812, 0.748876, -13.659928,
                0.748876, 4883.735840, 0.115151,
                -13.659928, 0.115151, 4522.395996])

mugI_scale_factor = 100
mugI = (mug_density*mugI/(mugI_scale_factor**5)).reshape(3,3)





def print_inertia_values(I):
    output = f"\n <ixx>{I[0,0]}</ixx>\n \
          <ixy>{I[0,1]}</ixy>\n \
          <ixz>{I[0,2]}</ixz>\n \
          <iyy>{I[1,1]}</iyy>\n \
          <iyz>{I[1,2]}</iyz>\n \
          <izz>{I[2,2]}</izz>"
    print(output)

def print_mass(volume, density):
    print(f"<mass>{volume*density}</mass>")



print_inertia_values(coffecanI)
print_mass(coffe_volume, coffe_can_density)

print_inertia_values(milkI)
print_mass(milk_volume, milk_density)


print_inertia_values(mugI)
print_mass(mug_volume, mug_density)

