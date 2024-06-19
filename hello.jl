println("Hello, World!")
function mandelbrot(a)
    z = 0
    for i=1:50
        z = z^2 + a
    end
    return z #test
end

for y=1.0:-0.05:-1.0
    for x=-2.0:0.0315:0.5
        abs(mandelbrot(complex(x, y))) < 2 ? print("*") : print(" ")
    end
    println()
end

1 + 1

2+3

5/2
# Taken from: https://rosettacode.org/wiki/Mandelbrot_set#Julia

2^3

5%2

3<2
2>1

x = 2

if x > 1
    println("x is greater than 1")
end
typeof(2.3)
typeof(2)

1/3
1//3

1//3 + 1//3

1/3 == 1//3 
sqrt(2)

 5 / 2

 5\ 2

 div(5, 2)

 typeof('a')
 typeof("world")
 println(""""Hello", World!""")

 #arrays 

 col_vec = [1, 2,3]
 col_vec_T = [1, 2,3]'

 and_this = [1 2 3]


 and_this == col_vec_T

 and_this[2]

 length(and_this)
 length(col_vec)

 sum(col_vec)
 sort(col_vec'; rev=false)

 println(and_this)

 test_vec = [5,7, 9]

 test_vec2 = [5; 7; 9]
 test_vec == test_vec2
 push!(test_vec, 4)

 test_tuple = ("house", 5, 3.14, 2//3)
 typeof(test_tuple)

 word2 = "Hello, World!"    

 for i in word2
    println(i)
 end

 function myadd(x,y)
    x + y
 end 

 myadd(2, 3)

 f(a,b) = sqrt(a^2 + b^2)

 f(3, 4)

 