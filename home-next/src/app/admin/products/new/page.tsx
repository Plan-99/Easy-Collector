import ProductForm from "../ProductForm";

export default function AdminNewProductPage() {
  return (
    <>
      <h2 className="font-bold text-lg mb-4">상품 추가</h2>
      <ProductForm
        initial={{
          sku: "",
          name: "",
          description: "",
          priceKrw: 0,
          stock: 0,
          weightG: null,
          leadTimeDays: null,
          imageUrl: "",
          active: true,
          variants: [],
        }}
      />
    </>
  );
}
