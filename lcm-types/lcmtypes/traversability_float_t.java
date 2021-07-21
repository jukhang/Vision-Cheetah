/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package lcmtypes;
 
import java.io.*;
import java.util.*;
import lcm.lcm.*;
 
public final class traversability_float_t implements lcm.lcm.LCMEncodable
{
    public float map[][];
 
    public traversability_float_t()
    {
        map = new float[100][100];
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0x30fdeed0a0459083L;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class<?>>());
    }
 
    public static long _hashRecursive(ArrayList<Class<?>> classes)
    {
        if (classes.contains(lcmtypes.traversability_float_t.class))
            return 0L;
 
        classes.add(lcmtypes.traversability_float_t.class);
        long hash = LCM_FINGERPRINT_BASE
            ;
        classes.remove(classes.size() - 1);
        return (hash<<1) + ((hash>>63)&1);
    }
 
    public void encode(DataOutput outs) throws IOException
    {
        outs.writeLong(LCM_FINGERPRINT);
        _encodeRecursive(outs);
    }
 
    public void _encodeRecursive(DataOutput outs) throws IOException
    {
        for (int a = 0; a < 100; a++) {
            for (int b = 0; b < 100; b++) {
                outs.writeFloat(this.map[a][b]); 
            }
        }
 
    }
 
    public traversability_float_t(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public traversability_float_t(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static lcmtypes.traversability_float_t _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        lcmtypes.traversability_float_t o = new lcmtypes.traversability_float_t();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.map = new float[(int) 100][(int) 100];
        for (int a = 0; a < 100; a++) {
            for (int b = 0; b < 100; b++) {
                this.map[a][b] = ins.readFloat();
            }
        }
 
    }
 
    public lcmtypes.traversability_float_t copy()
    {
        lcmtypes.traversability_float_t outobj = new lcmtypes.traversability_float_t();
        outobj.map = new float[(int) 100][(int) 100];
        for (int a = 0; a < 100; a++) {
            System.arraycopy(this.map[a], 0, outobj.map[a], 0, 100);        }
 
        return outobj;
    }
 
}
