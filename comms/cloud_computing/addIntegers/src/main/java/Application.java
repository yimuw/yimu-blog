import java.io.*;
import java.math.BigInteger;

public class Application {
    public static void main(String[] args) throws FileNotFoundException
    {
        BigInteger start = new BigInteger(args[0]);
        BigInteger end = new BigInteger(args[1]);
        
        BigInteger sum = new BigInteger("0");
        for(; start.compareTo(end) < 0; start = start.add(BigInteger.ONE)) {
            sum = sum.add(start);
        }
        System.out.println("sum: " + sum);

        PrintWriter p = new PrintWriter(new FileOutputStream("output.txt", false));
        p.println(sum.toString());
        p.close();
        System.out.println("file saved");
    }
}
