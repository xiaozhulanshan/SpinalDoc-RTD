xxx Register Interface Builder

- Automatic address, fields allocation and conflict detection
- 28 Register Access types (Covering the 25 types defined by the UVM standard)
- Automatic documentation generation

Automatic allocation
====================

Automatic address allocation

.. code:: scala

  class RegBankExample extends Component {
    val io = new Bundle {
      apb = slave(Apb3(Apb3Config(16,32)))
    }
    val busif = Apb3BusInterface(io.apb,(0x0000, 100 Byte))
    val M_REG0  = busif.newReg(doc="REG0")
    val M_REG1  = busif.newReg(doc="REG1")
    val M_REG2  = busif.newReg(doc="REG2")

    val M_REGn  = busif.newRegAt(address=0x40, doc="REGn")
    val M_REGn1 = busif.newReg(doc="REGn1")

    busif.accept(HtmlGenerator("regif", "AP"))
    // busif.accept(DocCHeader("header", "AP"))
    // busif.accept(DocJson("regif"))
    // busif.accept(DocRalf("regbank"))
    // busif.accept(DocSystemRdl("regif"))
  }

.. image:: /asset/image/regif/reg-auto-allocate.gif

Automatic fileds allocation

.. code:: scala

  val M_REG0  = busif.newReg(doc="REG1")
  val fd0 = M_REG0.field(Bits(2 bit), RW, doc= "fields 0")
  M_REG0.reserved(5 bits)
  val fd1 = M_REG0.field(Bits(3 bit), RW, doc= "fields 0")
  val fd2 = M_REG0.field(Bits(3 bit), RW, doc= "fields 0")
  //auto reserved 2 bits
  val fd3 = M_REG0.fieldAt(pos=16, Bits(4 bit), doc= "fields 3")
  //auto reserved 12 bits

.. image:: /asset/image/regif/field-auto-allocate.gif

conflict detection

.. code:: scala

  val M_REG1  = busif.newReg(doc="REG1")
  val r1fd0 = M_REG1.field(Bits(16 bits), RW, doc="fields 1")
  val r1fd2 = M_REG1.field(Bits(18 bits), RW, doc="fields 1")
    ...
  cause Exception
  val M_REG1  = busif.newReg(doc="REG1")
  val r1fd0 = M_REG1.field(Bits(16 bits), RW, doc="fields 1")
  val r1fd2 = M_REG1.fieldAt(pos=10, Bits(2 bits), RW, doc="fields 1")
    ...
  cause Exception
