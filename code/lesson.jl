println("Hello, world")
println("julia works")

my_answer=42
typeof(my_answer)

#this is how i comment

#=
For multi comment i do this
sequence
=#

#string
s1= "I am string"
s2="""I am also a string"""

#= we can use the $ sign to insert existing variables into a string and evaluate expressions withing the string=#

name="Jane"
println("Hello,my name is $name")
myphonebook = Dict("Jenny"=> "867-5309", "Ghostbusters" => "555-2368")
myphonebook["Kramer"]= "555-FILK"
myphonebook["Kramer"]
FIBONACCI = [1,1,2, 4, 5]
push!(FIBONACCI,21)
rand(4,3)
#loops
#=m = 0
while m<10
    m += 1
    println(m)
end=#

for n in 1:10 
    println(n)
end

m,n = 5,5
A=zeros(m, n)

x=3
y=90

if x>y
    println("$x is larger than $y")
elseif y>x
    println("$y is larger than $x")
else 
    println("$x and $y are equal")
end
(x>y) ? x : y
#declare function
function sayhi(name)
    println("Hi $name it s great to see you")
end
sayhi("c")
#Mutating vs nonmutating
v=[3,5,2]
sort!(v)
#=Broadcast By placing a . between any function name and its argument list, we tell that function to broadcast over 
the elements of the input objects=#
A=[i+3*j for j in 0:2, i in 1:3]
f(A)=A^2

B=f.(A)

