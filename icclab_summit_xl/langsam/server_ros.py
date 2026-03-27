"""
LangSAM Server for ROS Integration

This server provides both image and JSON endpoints for segmentation:
- /predict: Returns rendered PNG images (compatible with original Gradio app)
- /predict_json: Returns raw segmentation data (masks, boxes, scores, labels) as JSON

The JSON endpoint is designed for ROS nodes that need raw segmentation data
to generate point clouds and perform further processing.

IMPORTANT: Run this from the lang-segment-anything root directory:
    cd ~/rap/lang-segment-anything
    python3 -m lang_sam.server_ros

Do NOT run it as:
    python3 lang_sam/server_ros.py
This will cause module import errors.
"""
from io import BytesIO
import base64

import litserve as ls
import numpy as np
from fastapi import Response, UploadFile
from PIL import Image

from lang_sam import LangSAM
from lang_sam.utils import draw_image

PORT = 8001  # Different port to avoid conflict with original server


class LangSAMAPI(ls.LitAPI):
    def setup(self, device: str) -> None:
        """Initialize or load the LangSAM model."""
        # Using grounding-dino-base for better accuracy (larger than tiny)
        self.model = LangSAM(
            sam_type="sam2.1_hiera_small",
            gdino_model_id="IDEA-Research/grounding-dino-base",
            device=device
        )
        print("LangSAM model initialized with Grounding DINO Base.")

    def decode_request(self, request) -> dict:
        """Decode the incoming request to extract parameters and image bytes.

        Assumes the request is sent as multipart/form-data with fields:
        - sam_type: str
        - box_threshold: float
        - text_threshold: float
        - text_prompt: str
        - image: UploadFile
        - output_format: str (optional, 'image' or 'json', default 'image')
        """
        # Extract form data
        sam_type = request.get("sam_type")
        box_threshold = float(request.get("box_threshold", 0.3))
        text_threshold = float(request.get("text_threshold", 0.25))
        text_prompt = request.get("text_prompt", "")
        output_format = request.get("output_format", "image")

        # Extract image file
        image_file: UploadFile = request.get("image")
        if image_file is None:
            raise ValueError("No image file provided in the request.")

        image_bytes = image_file.file.read()

        return {
            "sam_type": sam_type,
            "box_threshold": box_threshold,
            "text_threshold": text_threshold,
            "image_bytes": image_bytes,
            "text_prompt": text_prompt,
            "output_format": output_format,
        }

    def predict(self, inputs: dict) -> dict:
        """Perform prediction using the LangSAM model.

        Returns:
            dict: Contains either output_image (PIL) or raw_results (dict) depending on output_format
        """
        print("Starting prediction with parameters:")
        print(
            f"sam_type: {inputs['sam_type']}, "
            f"box_threshold: {inputs['box_threshold']}, "
            f"text_threshold: {inputs['text_threshold']}, "
            f"text_prompt: {inputs['text_prompt']}, "
            f"output_format: {inputs['output_format']}"
        )

        if inputs["sam_type"] and inputs["sam_type"] != self.model.sam_type:
            print(f"Updating SAM model type to {inputs['sam_type']}")
            self.model.sam.build_model(inputs["sam_type"])

        try:
            image_pil = Image.open(BytesIO(inputs["image_bytes"])).convert("RGB")
        except Exception as e:
            raise ValueError(f"Invalid image data: {e}")

        results = self.model.predict(
            images_pil=[image_pil],
            texts_prompt=[inputs["text_prompt"]],
            box_threshold=inputs["box_threshold"],
            text_threshold=inputs["text_threshold"],
        )
        results = results[0]

        if not len(results["masks"]):
            print("No masks detected.")
            if inputs["output_format"] == "json":
                return {
                    "output_format": "json",
                    "raw_results": {
                        "masks": [],
                        "boxes": [],
                        "scores": [],
                        "labels": [],
                        "image_shape": list(image_pil.size[::-1]) + [3],  # (H, W, C)
                    }
                }
            else:
                return {"output_format": "image", "output_image": image_pil}

        # Return raw results for JSON format
        if inputs["output_format"] == "json":
            # Convert numpy arrays to lists for JSON serialization
            json_results = {
                "masks": [],
                "boxes": results["boxes"].tolist(),
                "scores": results["scores"].tolist(),
                "labels": results["labels"],
                "image_shape": list(image_pil.size[::-1]) + [3],  # (H, W, C)
            }

            # Encode masks as base64 compressed numpy arrays
            for mask in results["masks"]:
                # Convert boolean mask to uint8 for efficient storage
                mask_uint8 = mask.astype(np.uint8)
                # Compress and encode
                mask_bytes = mask_uint8.tobytes()
                mask_b64 = base64.b64encode(mask_bytes).decode('utf-8')
                json_results["masks"].append({
                    "data": mask_b64,
                    "shape": mask.shape,
                    "dtype": "uint8"
                })

            return {"output_format": "json", "raw_results": json_results}

        # Draw results on the image for image format
        image_array = np.asarray(image_pil)
        output_image = draw_image(
            image_array,
            results["masks"],
            results["boxes"],
            results["scores"],
            results["labels"],
        )
        output_image = Image.fromarray(np.uint8(output_image)).convert("RGB")

        return {"output_format": "image", "output_image": output_image}

    def encode_response(self, output: dict) -> Response:
        """Encode the prediction result into an HTTP response.

        Returns:
            Response: Contains either PNG image or JSON data
        """
        if output["output_format"] == "json":
            import json
            return Response(
                content=json.dumps(output["raw_results"]),
                media_type="application/json"
            )
        else:
            try:
                image = output["output_image"]
                buffer = BytesIO()
                image.save(buffer, format="PNG")
                buffer.seek(0)
                return Response(content=buffer.getvalue(), media_type="image/png")
            except Exception as e:
                raise ValueError(f"No output generated by the prediction: {e}")


lit_api = LangSAMAPI()
server = ls.LitServer(lit_api, api_path="/predict")


if __name__ == "__main__":
    print(f"Starting LangSAM ROS Server on port {PORT}...")
    print(f"Endpoints:")
    print(f"  - POST http://localhost:{PORT}/predict (output_format=image for PNG)")
    print(f"  - POST http://localhost:{PORT}/predict (output_format=json for JSON)")
    server.run(port=PORT)
